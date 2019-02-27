// #define ZERYNTH_PRINTF
//conflict with register names in nrf52 -_-
#define Z_VHAL_SKIP_PIN_DEFS

#include "zerynth.h"
#include "ble_ifc.h"


#define FIRST_CONN_PARAMS_UPDATE_DELAY      5000                                    /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY       30000                                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT        3                                       /**< Number of attempts before giving up the connection parameter negotiation. */


extern void* vm_m_gatt;
extern void* vm_m_scan;
BLEDevice ble;

static uint16_t  m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */
static uint8_t   m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static pm_peer_id_t m_peer_to_be_deleted = PM_PEER_ID_INVALID;
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = ble.advert_data,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = ble.scanrsp_data,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

    }
};
static VSemaphore bleevt=NULL;


int _ble_get_scr(BLEScanResult *bscr){
    int filled = 0;
    vosSysLockIsr();
    if(ble.scan_n<=0) {
        //empty ring, nothing to do...
    } else {
        int pos = (NUM_SCAN_EVTS+ble.scan_p-ble.scan_n)%NUM_SCAN_EVTS;
        *bscr = ble.scr[pos];
        // memcpy(blee,&ble.events[pos],sizeof(BLEEvent));
        ble.scan_n--;
        filled=1;
    }
    vosSysUnlockIsr();
    return filled;
}

int _ble_put_scr(BLEScanResult *bscr){
    int new_in =0 ;
    vosSysLockIsr();
    ble.scan_n++;
    if(ble.scan_n>NUM_SCAN_EVTS) {
        //can't add new scan result without losing oldest one
        //not adding!
        ble.scan_n=NUM_SCAN_EVTS;
    } else {
        new_in=1;
        // memcpy(&ble.events[ble.evt_p],blee,sizeof(BLEEvent));
        ble.scr[ble.scan_p]=*bscr;
        ble.scan_p=(ble.scan_p+1)%NUM_SCAN_EVTS;
    }
    vosSysUnlockIsr();
    return new_in;
}



int _ble_get_evt(BLEEvent *blee){
    int filled = 0;
    vosSysLockIsr();
    if(ble.evt_n<=0) {
        //empty ring, nothing to do...
    } else {
        int pos = (NUM_BLE_EVTS+ble.evt_p-ble.evt_n)%NUM_BLE_EVTS;
        *blee = ble.events[pos];
        // memcpy(blee,&ble.events[pos],sizeof(BLEEvent));
        ble.evt_n--;
        filled=1;
    }
    vosSysUnlockIsr();
    return filled;
}

int _ble_put_evt(BLEEvent *blee){
    int new_in =0 ;
    vosSysLockIsr();
    ble.evt_n++;
    if(ble.evt_n>NUM_BLE_EVTS) {
        //can't add new event without losing oldest one
        //not adding!
        ble.evt_n=NUM_BLE_EVTS;
    } else {
        new_in=1;
        // memcpy(&ble.events[ble.evt_p],blee,sizeof(BLEEvent));
        ble.events[ble.evt_p]=*blee;
        ble.evt_p=(ble.evt_p+1)%NUM_BLE_EVTS;
    }
    vosSysUnlockIsr();
    if (new_in) vosSemSignal(bleevt);
    return new_in;
}


/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id){
         case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT: {
            BLEEvent blee = {BLE_GAP_EVENT,BLE_GAP_EVT_SCAN_STOPPED,0,0};
            _ble_put_evt(&blee);
         } break;
    }
}


/**@brief Function for initializing the scanning and setting the filters.
 */
static int scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    if(!ble.scanning_set){
        memset(&ble.scan_param,0,sizeof(ble_gap_scan_params_t));
        ble.scan_param.active=0;
        ble.scan_param.filter_policy=BLE_GAP_SCAN_FP_ACCEPT_ALL;
        ble.scan_param.scan_phys = BLE_GAP_PHY_AUTO;
        ble.scan_param.window = 10*1000/625;
        ble.scan_param.interval = 10*1000/625;
        ble.scan_param.timeout = 0;
    }

    init_scan.p_scan_param = &ble.scan_param;
    init_scan.connect_if_match = false;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&vm_m_scan, &init_scan, scan_evt_handler);
    SDH_ERROR_CHECK(err_code,"failed scan init");
    return 0;
}



static int pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    printf("PM_EVT %x\n",p_evt->evt_id);
    switch(p_evt->evt_id){
       case PM_EVT_BONDED_PEER_CONNECTED:           printf("PM_EVT_BONDED_PEER_CONNECTED\n"); break;
       case PM_EVT_CONN_SEC_START:                  printf("PM_EVT_CONN_SEC_START\n"); break;
       case PM_EVT_CONN_SEC_SUCCEEDED:{
            printf("PM_EVT_CONN_SEC_SUCCEEDED %x %x\n",p_evt->params.conn_sec_succeeded.procedure,p_evt->params.conn_sec_succeeded.data_stored);
            pm_conn_sec_status_t conn_sec_status;
            int conn_ok=1;
            m_peer_to_be_deleted = PM_PEER_ID_INVALID;

            err_code = pm_conn_sec_status_get(p_evt->conn_handle, &conn_sec_status);
            if(err_code!=NRF_SUCCESS) conn_ok=0;
            if(ble.scheme&AUTH_MITM && !conn_sec_status.mitm_protected) conn_ok=0;
            if(ble.scheme&AUTH_SC && !conn_sec_status.encrypted) conn_ok=0;
            if(ble.bonding && !conn_sec_status.bonded) {
                conn_ok=0;
            }

            if (!conn_ok){
                //disconnect and optionally unbond
                if(conn_sec_status.bonded && ble.bonding) {
                    //unbond!
                    err_code = pm_peer_id_get(m_conn_handle, &m_peer_to_be_deleted);
                    err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                }
            }
        }
            break;
       case PM_EVT_CONN_SEC_FAILED:                 {
            printf("PM_EVT_CONN_SEC_FAILED %x %x %x\n",p_evt->params.conn_sec_failed.procedure,p_evt->params.conn_sec_failed.error,p_evt->peer_id);
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_AUTHENTICATION_FAILURE);
            break;
        }
       case PM_EVT_CONN_SEC_CONFIG_REQ:             printf("PM_EVT_CONN_SEC_CONFIG_REQ\n"); break;
       case PM_EVT_CONN_SEC_PARAMS_REQ:             printf("PM_EVT_CONN_SEC_PARAMS_REQ\n"); break;
       case PM_EVT_STORAGE_FULL:                    printf("PM_EVT_STORAGE_FULL\n"); break;
       case PM_EVT_ERROR_UNEXPECTED:                printf("PM_EVT_ERROR_UNEXPECTED\n"); break;
       case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:      printf("PM_EVT_PEER_DATA_UPDATE_SUCCEEDED\n"); break;
       case PM_EVT_PEER_DATA_UPDATE_FAILED:         printf("PM_EVT_PEER_DATA_UPDATE_FAILED\n"); break;
       case PM_EVT_PEER_DELETE_SUCCEEDED:           printf("PM_EVT_PEER_DELETE_SUCCEEDED\n"); ble.removed_bond=1;break;
       case PM_EVT_PEER_DELETE_FAILED:              printf("PM_EVT_PEER_DELETE_FAILED\n"); ble.removed_bond=2;break;
       case PM_EVT_PEERS_DELETE_SUCCEEDED:          printf("PM_EVT_PEERS_DELETE_SUCCEEDED\n"); break;
       case PM_EVT_PEERS_DELETE_FAILED:             printf("PM_EVT_PEERS_DELETE_FAILED\n"); break;
       case PM_EVT_LOCAL_DB_CACHE_APPLIED:          printf("PM_EVT_LOCAL_DB_CACHE_APPLIED\n"); break;
       case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:     printf("PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED\n"); break;
       case PM_EVT_SERVICE_CHANGED_IND_SENT:        printf("PM_EVT_SERVICE_CHANGED_IND_SENT\n"); break;
       case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:   printf("PM_EVT_SERVICE_CHANGED_IND_CONFIRMED\n"); break;
       case PM_EVT_SLAVE_SECURITY_REQ:              printf("PM_EVT_SLAVE_SECURITY_REQ\n"); break;
       case PM_EVT_FLASH_GARBAGE_COLLECTED:         printf("PM_EVT_FLASH_GARBAGE_COLLECTED\n"); break;
       case PM_EVT_FLASH_GARBAGE_COLLECTION_FAILED: printf("PM_EVT_FLASH_GARBAGE_COLLECTION_FAILED\n"); break;
    }
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);
}


static int peer_manager_init(){
    ret_code_t           err_code;

    err_code = pm_init();
    SDH_ERROR_CHECK(err_code,"pm init");
    if(!ble.security_set) {
        //set defaults
        memset(&ble.sec_param, 0, sizeof(ble_gap_sec_params_t));
        // Security parameters to be used for all security procedures.
        ble.sec_param.bond           = SEC_PARAM_BOND;
        ble.sec_param.mitm           = SEC_PARAM_MITM;
        ble.sec_param.lesc           = SEC_PARAM_LESC;
        ble.sec_param.keypress       = SEC_PARAM_KEYPRESS;
        ble.sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
        ble.sec_param.oob            = SEC_PARAM_OOB;
        ble.sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
        ble.sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
        ble.sec_param.kdist_own.enc  = 0;
        ble.sec_param.kdist_own.id   = 0;
        ble.sec_param.kdist_peer.enc = 0;
        ble.sec_param.kdist_peer.id  = 0;

    }

    printf("Setting cap %i\n",ble.sec_param.io_caps);
    err_code = pm_sec_params_set(&ble.sec_param);
    SDH_ERROR_CHECK(err_code,"sec param");

    err_code = pm_register(pm_evt_handler);
    SDH_ERROR_CHECK(err_code,"pm register");
    return 0;
}


static int gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_sec_mode_t sec_mode;

    if (!ble.conn_set){
        memset(&ble.conn_params, 0, sizeof(ble.conn_params));

        ble.conn_params.min_conn_interval = MIN_CONN_INTERVAL;
        ble.conn_params.max_conn_interval = MAX_CONN_INTERVAL;
        ble.conn_params.slave_latency     = SLAVE_LATENCY;
        ble.conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    } else {
        if (ble.security==1){
            switch(ble.level){
                case 1:
                    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
                    break;
                case 2:
                    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&sec_mode);
                    break;
                case 3:
                    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&sec_mode);
                    break;
                case 4:
                    BLE_GAP_CONN_SEC_MODE_SET_LESC_ENC_WITH_MITM(&sec_mode);
                    break;

            }
        }else {
            if (ble.level==1){
                BLE_GAP_CONN_SEC_MODE_SET_SIGNED_NO_MITM(&sec_mode);
            } else {
                BLE_GAP_CONN_SEC_MODE_SET_SIGNED_WITH_MITM(&sec_mode);
            }
        }
    }

    if (ble.namelen){
        err_code = sd_ble_gap_device_name_set(&sec_mode,(const uint8_t *)ble.name,ble.namelen);
        SDH_ERROR_CHECK(err_code,"failed gap name set");
    }

    err_code = sd_ble_gap_appearance_set(ble.appearance);
    SDH_ERROR_CHECK(err_code,"failed gap appearance set");

    err_code = sd_ble_gap_ppcp_set(&ble.conn_params);
    SDH_ERROR_CHECK(err_code,"failed fap set parameters");
    return 0;
}




static int advertising_init(void)
{
    ret_code_t             err_code;
    ble_gap_adv_params_t adv_params;


    //initialize params
    memset(&adv_params, 0, sizeof(adv_params));
    if (ble.adv_mode==0) adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    else if (ble.adv_mode==2) adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED;
    else if (ble.adv_mode==3) adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;

    adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    adv_params.interval        = (ble.advertising_interval) ? (ble.advertising_interval):(APP_ADV_INTERVAL);
    adv_params.duration        = ble.advertising_timeout;

    m_adv_data.adv_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;
    m_adv_data.scan_rsp_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;

    if(ble.advert_data_len) {
        //custom adv data
        m_adv_data.adv_data.len = ble.advert_data_len;
        m_adv_data.scan_rsp_data.len = ble.scanrsp_data_len;

    } else {
        //non custom adv data
        ble_advdata_t           advdata;

        memset(&advdata, 0, sizeof(advdata));
        if (ble.namelen) {
            advdata.name_type               = BLE_ADVDATA_FULL_NAME;
        } else {
            advdata.name_type               = BLE_ADVDATA_NO_NAME;
        }
        advdata.include_appearance      = (ble.appearance==0) ?false:true;
        advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

        err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
        SDH_ERROR_CHECK(err_code,"failed encode");

        if(ble.scanrsp_data_len) {
            m_adv_data.scan_rsp_data.len = ble.scanrsp_data_len;
        }
    }

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    SDH_ERROR_CHECK(err_code,"failed configure");
}

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt){
    // uint32_t err_code;
    uint32_t err_code;
    printf("CONN %x\n",p_evt->evt_type);

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED){
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        // APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    printf("CONN ERROR %x\n",nrf_error);
    //APP_ERROR_HANDLER(nrf_error);
}


static int conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = &ble.conn_params;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    SDH_ERROR_CHECK(err_code, "failed conn params");
    return 0;
    // APP_ERROR_CHECK(err_code);
}

static int ble_find_ch_by_uuid(uint16_t uuid, uint16_t srv_uuid, BLECharacteristic **rch){
    int i,j;

    for(i=0;i<ble.nserv;i++){
        BLEService *s = &SERVICE(i);
        if (s->uuid!=srv_uuid && srv_uuid) {
            //not this service
            continue;
        }
        for(j=0;j<s->nchar;j++){
            BLECharacteristic *ch = &CHARACTERISTIC(i,j);
            if(ch->uuid==uuid) {
                *rch = ch;
                return 0;
            }
        }
    }
    return -1;
}

static int ble_find_ch_by_handle(uint16_t handle, BLECharacteristic **rch){
    int i,j;

    for(i=0;i<ble.nserv;i++){
        BLEService *s = &SERVICE(i);
        for(j=0;j<s->nchar;j++){
            BLECharacteristic *ch = &CHARACTERISTIC(i,j);
            if(ch->handles.value_handle==handle) {
                *rch = ch;
                return 1;
            } else if (ch->handles.cccd_handle){
                *rch=ch;
                return 2;
            }
        }
    }
    return -1;
}

static int ble_write_ch(uint16_t handle, uint8_t* data, uint16_t size, uint16_t offset){
    BLECharacteristic *ch = NULL;
    int res = ble_find_ch_by_handle(handle,&ch);
    if(res<0) return res;
    if(res==1) {
        //it's a value handle
        if(ch->perm & BLE_PERM_WRITE) {
            if(ch->max_vsize>=size) {
                //ignore offset
                memcpy(ch->value,data,size);
                ch->vsize = size;
                BLEEvent blee = {BLE_GATT_EVENT,8,ch->service->uuid,ch->uuid};
                _ble_put_evt(&blee);
            }
        }
    } else if(res==2) {
        //it's a cccd handle
        if (size==2) {
            //accept only 2 bytes writes
            uint16_t cccd_val = ((uint16_t*)data)[0];
            printf("cccd_val %x\n",cccd_val);
            if (cccd_val==0x0001) {
                //notify
                if (ch->perm & BLE_PERM_NOTIFY) {
                    ch->cccd |= BLE_PERM_NOTIFY;
                }
            } else if(cccd_val==0x0002) {
                //indicate
                if (ch->perm & BLE_PERM_INDICATE) {
                    ch->cccd |=BLE_PERM_INDICATE;
                }
            } else if(cccd_val=0x0000){
                ch->cccd = 0;
            } else {
                //bad format
            }
        }
    }
    return res;
}
static void on_ble_gatts_event(ble_evt_t const * p_ble_evt, void * p_context){
    ret_code_t err_code;
    switch (p_ble_evt->header.evt_id){
        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            printf("GATT Server Timeout.\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            // APP_ERROR_CHECK(err_code);
            break;
        case BLE_GATTS_EVT_WRITE: {
                if (m_conn_handle==BLE_CONN_HANDLE_INVALID) break;
                uint16_t handle = p_ble_evt->evt.gatts_evt.params.write.handle;
                uint8_t *data = p_ble_evt->evt.gatts_evt.params.write.data;
                uint16_t size = p_ble_evt->evt.gatts_evt.params.write.len;
                uint16_t offset = p_ble_evt->evt.gatts_evt.params.write.offset;
                ble_write_ch(handle,data,size,offset);
            }
            break;
        default:
            break;
    }
}

static void on_ble_gattc_event(ble_evt_t const * p_ble_evt, void * p_context){
    ret_code_t err_code;
    switch (p_ble_evt->header.evt_id){
        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            printf("GATT Client Timeout.\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            // APP_ERROR_CHECK(err_code);
            break;
        default:
            break;
    }
}

static void on_ble_gap_event(ble_evt_t const * p_ble_evt, void * p_context){
    ret_code_t err_code;
    switch (p_ble_evt->header.evt_id){
        case BLE_GAP_EVT_CONN_PARAM_UPDATE: printf("BLE_GAP_EVT_CONN_PARAM_UPDATE\n");break;
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST: {
                printf("BLE_GAP_EVT_SEC_PARAMS_REQUEST\n");
                printf("peer cap %i\n",p_ble_evt->evt.gap_evt.params.sec_params_request.peer_params.io_caps);
                break;
            }
        case BLE_GAP_EVT_SEC_INFO_REQUEST: printf("BLE_GAP_EVT_SEC_INFO_REQUEST\n");break;
        case BLE_GAP_EVT_PASSKEY_DISPLAY: {
                printf("BLE_GAP_EVT_PASSKEY_DISPLAY %i\n",p_ble_evt->evt.gap_evt.params.passkey_display.match_request);
                //the passkey is generated by softdevice! can't be decided by the programmer
                memcpy(ble.passkey, p_ble_evt->evt.gap_evt.params.passkey_display.passkey, 6);
                if (p_ble_evt->evt.gap_evt.params.passkey_display.match_request){
                    //MATCH REQUEST
                    BLEEvent blee = {BLE_GAP_EVENT,BLE_GAP_EVT_MATCH_PASSKEY,0,0};
                    _ble_put_evt(&blee);
                } else {
                    //SHOW REQUEST
                    BLEEvent blee = {BLE_GAP_EVENT,BLE_GAP_EVT_SHOW_PASSKEY,0,0};
                    _ble_put_evt(&blee);
                }
            }
            break;
        case BLE_GAP_EVT_KEY_PRESSED: printf("BLE_GAP_EVT_KEY_PRESSED\n");break;
        case BLE_GAP_EVT_AUTH_KEY_REQUEST: printf("BLE_GAP_EVT_AUTH_KEY_REQUEST\n");break;
        case BLE_GAP_EVT_LESC_DHKEY_REQUEST: printf("BLE_GAP_EVT_LESC_DHKEY_REQUEST\n");break;
        case BLE_GAP_EVT_CONN_SEC_UPDATE: printf("BLE_GAP_EVT_CONN_SEC_UPDATE %x %x\n",p_ble_evt->evt.gap_evt.params.conn_sec_update.conn_sec.sec_mode,p_ble_evt->evt.gap_evt.params.conn_sec_update.conn_sec.encr_key_size);break;
        case BLE_GAP_EVT_TIMEOUT: printf("BLE_GAP_EVT_TIMEOUT\n");break;
        case BLE_GAP_EVT_RSSI_CHANGED: printf("BLE_GAP_EVT_RSSI_CHANGED\n");break;
        case BLE_GAP_EVT_SEC_REQUEST: printf("BLE_GAP_EVT_SEC_REQUEST\n");break;
        case BLE_GAP_EVT_SCAN_REQ_REPORT: printf("BLE_GAP_EVT_SCAN_REQ_REPORT\n");break;
        case BLE_GAP_EVT_PHY_UPDATE: printf("BLE_GAP_EVT_PHY_UPDATE\n");break;
        case BLE_GAP_EVT_CONNECTED:{
                m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
                printf("Connected %i\n",m_conn_handle);
                //copy peer addr
                memcpy(&ble.peer_addr,&p_ble_evt->evt.gap_evt.params.connected.peer_addr, sizeof(ble.peer_addr));
                ble.connected = 1;
                BLEEvent blee = {BLE_GAP_EVENT,BLE_GAP_EVT_IS_CONNECTED,0,0};
                _ble_put_evt(&blee);
                //TODO: set CCCD to 0
            }
            break;

        case BLE_GAP_EVT_DISCONNECTED:{
                printf("BLE_GAP_EVT_DISCONNECTED\n");
                printf("Disconnected\h");
                if (m_peer_to_be_deleted != PM_PEER_ID_INVALID){
                    err_code = pm_peer_delete(m_peer_to_be_deleted);
                    m_peer_to_be_deleted = PM_PEER_ID_INVALID;
                }
                ble.connected = 0;
                m_conn_handle = BLE_CONN_HANDLE_INVALID;
                BLEEvent blee = {BLE_GAP_EVENT,BLE_GAP_EVT_IS_DISCONNECTED,0,0};
                _ble_put_evt(&blee);
            }
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:{
            printf("PHY update request.\n");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            // APP_ERROR_CHECK(err_code);
        } break;
        case BLE_GAP_EVT_AUTH_STATUS:
            printf("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                          p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                          p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
            // printf("AUTH %x %x\n",p_ble_evt->evt.gap_evt.params.auth_status.auth_status,p_ble_evt->evt.gap_evt.params.auth_status.error_src);
            break;
        case BLE_GAP_EVT_ADV_REPORT:{
            BLEScanResult scr;
            scr.rssi = p_ble_evt->evt.gap_evt.params.adv_report.rssi;
            memcpy(scr.addr, p_ble_evt->evt.gap_evt.params.adv_report.peer_addr.addr,6);
            memset(scr.data,0,32);
            memcpy(scr.data, p_ble_evt->evt.gap_evt.params.adv_report.data.p_data,p_ble_evt->evt.gap_evt.params.adv_report.data.len);
            scr.datalen = p_ble_evt->evt.gap_evt.params.adv_report.data.len;
            int sctype = 0;
            if(p_ble_evt->evt.gap_evt.params.adv_report.type.connectable && !p_ble_evt->evt.gap_evt.params.adv_report.type.directed) sctype = 0; //ADV_CONN_UND
            else if(p_ble_evt->evt.gap_evt.params.adv_report.type.connectable && p_ble_evt->evt.gap_evt.params.adv_report.type.directed) sctype = 1; //ADV_CONN_DIR
            else if(!p_ble_evt->evt.gap_evt.params.adv_report.type.connectable && !p_ble_evt->evt.gap_evt.params.adv_report.type.directed && p_ble_evt->evt.gap_evt.params.adv_report.type.scannable ) sctype = 2; //ADV_SCAN_UND
            else if(!p_ble_evt->evt.gap_evt.params.adv_report.type.connectable && !p_ble_evt->evt.gap_evt.params.adv_report.type.directed && !p_ble_evt->evt.gap_evt.params.adv_report.type.scannable ) sctype = 3; //ADV_UNCN_UND
            else sctype = 4;
            scr.scantype = sctype;
            scr.addrtype = p_ble_evt->evt.gap_evt.params.adv_report.peer_addr.addr_type;
            int is_dup = 0;
            if(!ble.duplicates){
                //ignore dups
                for(int i=0;i<NUM_SCAN_EVTS;i++){
                    if (memcmp(&scr,&ble.scr[i],sizeof(BLEScanDup))==0) {
                        is_dup=1;
                        break;
                    }
                }
            }
            if(!is_dup){
                _ble_put_scr(&scr);
                BLEEvent blee = {BLE_GAP_EVENT,BLE_GAP_EVT_SCAN_REPORT,0,0};
                _ble_put_evt(&blee);
            }

            }
            break;
        case BLE_GAP_EVT_ADV_SET_TERMINATED:{
                //Advertise terminated
                BLEEvent blee = {BLE_GAP_EVENT,BLE_GAP_EVT_ADV_STOPPED,0,0};
                _ble_put_evt(&blee);
            }
            break;
        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
\ */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{

    printf("BLE %x\n",p_ble_evt->header.evt_id);

    //call all handlers --> optimize
    on_ble_gap_event(p_ble_evt,p_context);
    on_ble_gattc_event(p_ble_evt,p_context);
    on_ble_gatts_event(p_ble_evt,p_context);

}


/********************* natives */

C_NATIVE(_ble_init) {
    C_NATIVE_UNWARN();
    ret_code_t err_code;
    BLEHooks hooks;

    hooks.h_ble_evt = ble_evt_handler;

    vm_ble_set_hooks(&hooks);

    err_code = vm_ble_stack_init();
    BLE_ERROR_CHECK(err_code,"ble stack init fail",ERR_HARDWARE_INITIALIZATION_ERROR);
    vm_ble_thread_start();

    return ERR_OK;
}




C_NATIVE(_ble_set_gap) {
    C_NATIVE_UNWARN();
    *res = MAKE_NONE();
    if (parse_py_args("siiiiiii", nargs, args, &ble.name, &ble.namelen,&ble.security,&ble.level,&ble.appearance,&ble.min_conn,&ble.max_conn,&ble.latency,&ble.conn_sup) != 8)
        return ERR_TYPE_EXC;
    //make a copy of name
    uint8_t *bname = gc_malloc(ble.namelen);
    memcpy(bname,ble.name,ble.namelen);
    ble.name=bname;
    ble.conn_set= 1;
    ble.conn_params.min_conn_interval = (ble.min_conn*1000)/1250;
    ble.conn_params.max_conn_interval = (ble.max_conn*1000)/1250;
    ble.conn_params.slave_latency     = ble.latency;
    ble.conn_params.conn_sup_timeout  = (ble.conn_sup*1000)/10000;

    return ERR_OK;
}



C_NATIVE(_ble_get_event) {
    C_NATIVE_UNWARN();
    BLEEvent blee;
    int pos;
    RELEASE_GIL();


    vosSemWait(bleevt);
    _ble_get_evt(&blee);

    PTuple *tpl = ptuple_new(5,NULL);
    PTUPLE_SET_ITEM(tpl,0,PSMALLINT_NEW(blee.type));
    PTUPLE_SET_ITEM(tpl,1,PSMALLINT_NEW(blee.type));
    PTUPLE_SET_ITEM(tpl,2,PSMALLINT_NEW(blee.type));
    PTUPLE_SET_ITEM(tpl,3,PSMALLINT_NEW(blee.status));
    PTUPLE_SET_ITEM(tpl,4,MAKE_NONE());

    if (blee.type==BLE_GATT_EVENT){
        //GATT EVENT
        PTUPLE_SET_ITEM(tpl,1,PSMALLINT_NEW(blee.service));
        PTUPLE_SET_ITEM(tpl,2,PSMALLINT_NEW(blee.characteristic));
    } else {
        //GAP EVENT
       if(blee.status==BLE_GAP_EVT_SCAN_REPORT) {
            //create Python report
            BLEScanResult scr;
            if(_ble_get_scr(&scr)){
                PTuple *tpp = ptuple_new(5,NULL);
                PTUPLE_SET_ITEM(tpp,0,PSMALLINT_NEW(scr.scantype));
                PTUPLE_SET_ITEM(tpp,1,PSMALLINT_NEW(scr.addrtype));
                PTUPLE_SET_ITEM(tpp,2,PSMALLINT_NEW(scr.rssi));
                PBytes *pdata = pbytes_new(scr.datalen,scr.data);
                PBytes *paddr = pbytes_new(6,scr.addr);
                PTUPLE_SET_ITEM(tpp,3,pdata);
                PTUPLE_SET_ITEM(tpp,4,paddr);
                PTUPLE_SET_ITEM(tpl,4,tpp);
            }
        } else if(blee.status==BLE_GAP_EVT_IS_CONNECTED || blee.status==BLE_GAP_EVT_IS_DISCONNECTED) {
            PBytes *paddr = pbytes_new(6,ble.peer_addr.addr);
            PTUPLE_SET_ITEM(tpl,4,paddr);
        } else if(blee.status==BLE_GAP_EVT_SHOW_PASSKEY){
            PTUPLE_SET_ITEM(tpl,4,pstring_new(6,ble.passkey));
        } else if(blee.status==BLE_GAP_EVT_MATCH_PASSKEY){
            PTUPLE_SET_ITEM(tpl,4,pstring_new(6,ble.passkey));
        }
    }
    *res = tpl;
    ACQUIRE_GIL();
    return ERR_OK;
}

C_NATIVE(_ble_get_value) {
    C_NATIVE_UNWARN();
    int i,j;
    uint32_t serv_uuid;
    uint32_t uuid;
    *res = MAKE_NONE();

    if (parse_py_args("ii", nargs, args, &serv_uuid, &uuid) != 2)
        return ERR_TYPE_EXC;

    BLECharacteristic *ch = NULL;
    int rr = ble_find_ch_by_uuid(uuid, serv_uuid, &ch);
    if(rr<0) {
        return ERR_OK;
    }

    ret_code_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;
    memset(&gatts_value, 0, sizeof(gatts_value));
    PBytes *pb = pbytes_new(ch->vsize,NULL);

    gatts_value.len     = ch->vsize;
    gatts_value.offset  = 0;
    gatts_value.p_value = ch->value;
    err_code = sd_ble_gatts_value_get(m_conn_handle,
                      ch->handles.value_handle,
                      &gatts_value);
    BLE_ERROR_CHECK(err_code, "failed read value", ERR_RUNTIME_EXC);

    memcpy(PSEQUENCE_BYTES(pb),ch->value,ch->vsize);

    *res=pb;

    return ERR_OK;
}

C_NATIVE(_ble_set_value) {
    C_NATIVE_UNWARN();
    uint32_t serv_uuid;
    uint32_t uuid;
    uint8_t* value;
    uint32_t valuelen;
    *res = MAKE_NONE();
    if (parse_py_args("iis", nargs, args, &serv_uuid, &uuid,&value, &valuelen) != 3)
        return ERR_TYPE_EXC;
    BLECharacteristic *ch = NULL;
    int rr = ble_find_ch_by_uuid(uuid, serv_uuid, &ch);
    if(rr<0) {
        return ERR_INDEX_EXC;
    }

    ble_gatts_value_t           gatts_value;
    memset(&gatts_value, 0, sizeof(gatts_value));

    if(valuelen>ch->max_vsize) valuelen = ch->max_vsize;
    memcpy(ch->value,value,valuelen);
    ch->vsize = valuelen;

    gatts_value.len     = valuelen;
    gatts_value.offset  = 0;
    gatts_value.p_value = ch->value;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID){
        ret_code_t err_code = sd_ble_gatts_value_set(m_conn_handle, ch->handles.value_handle, &gatts_value);
        BLE_ERROR_CHECK(err_code, "failed value set", ERR_RUNTIME_EXC);
        printf("cccd %x perm %x\n",ch->cccd,ch->perm);
        //check CCCD for indication/notification
        if ( (ch->perm & (BLE_PERM_NOTIFY|BLE_PERM_INDICATE)) && (ch->cccd & (BLE_PERM_NOTIFY | BLE_PERM_INDICATE))){
            // Notify host
            ble_gatts_hvx_params_t hvx_params;
            uint16_t hvlen = ch->vsize;
            memset(&hvx_params, 0, sizeof(hvx_params));

            hvx_params.handle = ch->handles.value_handle;
            hvx_params.type   = (ch->cccd & BLE_PERM_INDICATE) ? (BLE_GATT_HVX_INDICATION):(BLE_GATT_HVX_NOTIFICATION);
            hvx_params.offset = 0;
            hvx_params.p_len  = &hvlen;
            hvx_params.p_data = NULL; //the value in SoftDevice is used

            err_code = sd_ble_gatts_hvx(m_conn_handle, &hvx_params);
            //ignore errors
            // BLE_ERROR_CHECK(err_code, "failed value notification", ERR_RUNTIME_EXC);
        }
    }

    return ERR_OK;
}


C_NATIVE(_ble_set_advertising) {
    C_NATIVE_UNWARN();
    *res = MAKE_NONE();
    uint8_t *payload;
    uint32_t payloadlen;
    uint8_t *payload2;
    uint32_t payloadlen2;
    if (parse_py_args("iisis", nargs, args, &ble.advertising_interval,&ble.advertising_timeout,&payload,&payloadlen,&ble.adv_mode,&payload2,&payloadlen2) != 5)
        return ERR_TYPE_EXC;
    memcpy(ble.advert_data,payload,payloadlen);
    ble.advert_data_len = payloadlen;
    memcpy(ble.scanrsp_data,payload2,payloadlen2);
    ble.scanrsp_data_len = payloadlen2;
    if (payloadlen>31) ble.advert_data_len=31;
    if (payloadlen2>31) ble.scanrsp_data_len=31;

    ble.advertising_interval = (int)(ble.advertising_interval*1000/625); //in 0.625ms steps
    ble.advertising_timeout = ble.advertising_timeout/10; //in 10 ms steps
    return ERR_OK;
}


C_NATIVE(_ble_start_advertising) {
    C_NATIVE_UNWARN();
    advertising_init();
    ret_code_t err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    BLE_ERROR_CHECK(err_code,"failed advertising start", ERR_RUNTIME_EXC);
    BLEEvent blee = {BLE_GAP_EVENT,BLE_GAP_EVT_ADV_STARTED,0,0};
    _ble_put_evt(&blee);
    return ERR_OK;
}

C_NATIVE(_ble_stop_advertising) {
    C_NATIVE_UNWARN();
    ret_code_t err_code = sd_ble_gap_adv_stop(m_adv_handle);
    BLE_ERROR_CHECK(err_code,"failed advertising stop", ERR_RUNTIME_EXC);
    BLEEvent blee = {BLE_GAP_EVENT,BLE_GAP_EVT_ADV_STOPPED,0,0};
    _ble_put_evt(&blee);
    return ERR_OK;
}

C_NATIVE(_ble_set_scanning) {
    C_NATIVE_UNWARN();
    int interval,window,duplicates,filter,addr,active;
    if (parse_py_args("iiiiii", nargs, args, &interval,&window,&duplicates,&filter,&addr,&active) != 6)
        return ERR_TYPE_EXC;
    memset(&ble.scan_param,0,sizeof(ble_gap_scan_params_t));
    ble.scan_param.interval = (int)(interval*1000/625);
    ble.scan_param.window = (int)(window*1000/625);
    ble.scan_param.active = (!active) ?  0:1;
    ble.scan_param.timeout = 0;
    ble.scanning_set=1;
    ble.duplicates = duplicates;
    return ERR_OK;
}

C_NATIVE(_ble_start_scanning) {
    C_NATIVE_UNWARN();
    int duration;
    ret_code_t err;
    if (parse_py_args("i", nargs, args, &duration) != 1)
        return ERR_TYPE_EXC;

    //zero out the buffers
    memset(ble.scr,0,sizeof(BLEScanResult)*NUM_SCAN_EVTS);
    ble.scan_param.timeout = duration/10;
    //set parameters again
    nrf_ble_scan_params_set(&vm_m_scan,&ble.scan_param);
    err = nrf_ble_scan_start(&vm_m_scan);
    BLE_ERROR_CHECK(err,"failed scan start",ERR_RUNTIME_EXC);
    return ERR_OK;
}

C_NATIVE(_ble_stop_scanning) {
    C_NATIVE_UNWARN();
    nrf_ble_scan_stop();
    return ERR_OK;
}

C_NATIVE(_ble_set_security){
    C_NATIVE_UNWARN();
    int has_passkey;
    if (parse_py_args("iiiiiiii", nargs, args, &ble.capabilities,&ble.bonding,&ble.scheme,&ble.key_size,&ble.init_key,&ble.resp_key,&ble.oob,&has_passkey) != 8)
        return ERR_TYPE_EXC;

    memset(&ble.sec_param, 0, sizeof(ble_gap_sec_params_t));
    ble.sec_param.bond           = (ble.bonding&AUTH_BOND)?1:0;
    ble.sec_param.mitm           = (ble.scheme&AUTH_MITM) ? 1:0;
    ble.sec_param.lesc           = (ble.scheme&AUTH_SC) ? 1:0;
    ble.sec_param.keypress       = (ble.scheme&AUTH_KEYPRESS) ? 1:0;
    ble.sec_param.io_caps        = ble.capabilities;
    ble.sec_param.oob            = ble.oob;
    ble.sec_param.min_key_size   = ble.key_size;
    ble.sec_param.max_key_size   = ble.key_size;
    ble.sec_param.kdist_own.enc  = (ble.init_key&KEY_ENC) ? 1:0;
    ble.sec_param.kdist_own.id   = (ble.init_key&KEY_ID) ? 1:0;
    ble.sec_param.kdist_peer.enc = (ble.resp_key&KEY_ENC) ? 1:0;
    ble.sec_param.kdist_peer.id  = (ble.resp_key&KEY_ID) ? 1:0;

    printf("bonding %i %i %i %i\n",ble.sec_param.bond,ble.bonding,AUTH_BOND,ble.key_size);
    printf("mitm %i \n",ble.sec_param.mitm);
    printf("lesc %i \n",ble.sec_param.lesc);
    printf("keypress %i \n",ble.sec_param.keypress);
    printf("kdist %i %i %i %i \n",ble.sec_param.kdist_own.enc,ble.sec_param.kdist_own.id,ble.sec_param.kdist_peer.enc,ble.sec_param.kdist_peer.id);
    printf("cap %x\n",ble.sec_param.io_caps);
    ble.security_set=1;
    return ERR_OK;
}

C_NATIVE(_ble_set_services) {
    C_NATIVE_UNWARN();

    int i,j,h,c,e=0;
    PList *slist = args[0];
    ble.nserv = PSEQUENCE_ELEMENTS(slist);
    *res  = MAKE_NONE();

    if (!ble.nserv) {
        //return and keep db at null
        return ERR_OK;
    }

    ble.services = gc_malloc(sizeof(BLEService)*ble.nserv);

    //let's count handles
    for(i=0;i<ble.nserv;i++){
        PTuple *psrv = (PTuple*)PLIST_ITEM(slist,i); //tuple of uuid, characteristics
        PList *chs = PTUPLE_ITEM(psrv,1); //get characteristics
        int nchs = PSEQUENCE_ELEMENTS(chs);
        PObject *base_uuid = PTUPLE_ITEM(psrv,2);
        // if(base_uuid!=*res){  //not None
        //     //extended uuid!
        //     ble_uuid128_t buuid;
        //     memcpy(buuid.uuid128,PSEQUENCE_BYTES(base_uuid),16);
        //     sd_ble_uuid_vs_add(&buuid, &uuid_type);
        // }
        SERVICE(i).uuid = (uint16_t)INTEGER_VALUE(PTUPLE_ITEM(psrv,0));
        SERVICE(i).nchar = nchs;
        SERVICE(i).chars = gc_malloc(sizeof(BLECharacteristic)*nchs);
        for(j=0;j<nchs;j++){
            PTuple *ch = PLIST_ITEM(chs,j); //take jth char
            //TODO: add long uuid support in ble.py
            int uuid = (uint16_t)PSMALLINT_VALUE(PTUPLE_ITEM(ch,0)); //uuid
            int permission = (uint16_t)PSMALLINT_VALUE(PTUPLE_ITEM(ch,2)); //permission
            int ilen = (uint16_t)PSMALLINT_VALUE(PTUPLE_ITEM(ch,3)); //value size
            CHARACTERISTIC(i,j).perm = permission;
            CHARACTERISTIC(i,j).max_vsize = ilen;
            CHARACTERISTIC(i,j).vsize = ilen;
            CHARACTERISTIC(i,j).uuid = uuid;
            CHARACTERISTIC(i,j).service = &SERVICE(i);
        }
    }

    return ERR_OK;
}

C_NATIVE(_ble_del_bonded) {
    C_NATIVE_UNWARN();
    uint8_t *addr;
    uint32_t addrlen;
    *res = MAKE_NONE();
    if (parse_py_args("s", nargs, args, &addr,&addrlen) != 1)
        return ERR_TYPE_EXC;

    if(addrlen!=6) return ERR_TYPE_EXC;

    int dev_num = pm_peer_count();
    printf("xdev %i\n",dev_num);
    pm_peer_id_t pid = PM_PEER_ID_INVALID;
    int i,j;
    pid = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    for(i=0;i<dev_num;i++){
        pm_peer_data_bonding_t bdata;
        pm_peer_data_bonding_load(pid,&bdata);
        if(memcmp(addr,bdata.peer_ble_id.id_addr_info.addr,6)==0){
            //delete and break after polling
            ble.removed_bond=0;
            pm_peer_delete(pid);
            while(!ble.removed_bond){
                vosThSleep(TIME_U(10,MILLIS));
            }
            if(ble.removed_bond==2) return ERR_RUNTIME_EXC;
            break;
        }
    }
    return ERR_OK;
}

C_NATIVE(_ble_get_bonded) {
    C_NATIVE_UNWARN();
    int dev_num = pm_peer_count();
    printf("dev %i\n",dev_num);
    pm_peer_id_t pid = PM_PEER_ID_INVALID;
    int i=0;
    PTuple *tpl = ptuple_new(dev_num,NULL);
    pid = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    for(i=0;i<dev_num;i++){
        pm_peer_data_bonding_t bdata;
        pm_peer_data_bonding_load(pid,&bdata);
        PBytes *pb = pbytes_new(6,bdata.peer_ble_id.id_addr_info.addr);
        PTuple *tpp = ptuple_new(2,NULL);
        PTUPLE_SET_ITEM(tpp,0,pb);
        PTUPLE_SET_ITEM(tpp,1,PSMALLINT_NEW(bdata.peer_ble_id.id_addr_info.addr_type));
        PTUPLE_SET_ITEM(tpl,i,tpp);
        pid = pm_next_peer_id_get(pid);
    }

    *res=tpl;
    return ERR_OK;
}

C_NATIVE(_ble_confirm_passkey) {
    C_NATIVE_UNWARN();
    *res = MAKE_NONE();
    uint32_t confirm;
    if (parse_py_args("i", nargs, args, &confirm) != 1)
        return ERR_TYPE_EXC;
    if(!confirm){
        sd_ble_gap_auth_key_reply(m_conn_handle,BLE_GAP_AUTH_KEY_TYPE_NONE,NULL);
    } else {
        sd_ble_gap_auth_key_reply(m_conn_handle,BLE_GAP_AUTH_KEY_TYPE_PASSKEY,NULL);
    }
    return ERR_OK;
}

ret_code_t services_init(){
    int i,j;
    ret_code_t   err;
    ble_uuid_t ble_uuid;

    if(!ble.services) return 0;


    for (i=0;i<ble.nserv;i++){
        BLEService *srv = &SERVICE(i);
         // Add service
        ble_uuid.type = BLE_UUID_TYPE_BLE;  //TODO: add support for 128bit uuid in ble.py
        ble_uuid.uuid = srv->uuid;

        printf("uuid %x\n",srv->uuid);
        err = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &srv->handle);
        SDH_ERROR_CHECK(err,"failed adding service");

        for(j=0;j<srv->nchar;j++){
            BLECharacteristic *ch = &CHARACTERISTIC(i,j);
            //add characteristic
            ble_add_char_params_t  add_char_params;
            ble_add_descr_params_t add_descr_params;

            memset(&add_char_params, 0, sizeof(add_char_params));
            add_char_params.uuid              = ch->uuid;
            add_char_params.max_len           = ch->max_vsize;
            add_char_params.init_len          = ch->vsize;
            add_char_params.p_init_value      = ch->value;
            add_char_params.is_value_user     = 1;// SEC_OPEN;//p_bas_init->bl_rd_sec;
            // add_char_params.is_val     = 1;// SEC_OPEN;//p_bas_init->bl_rd_sec;

            add_char_params.char_props.indicate       = (ch->perm&BLE_PERM_INDICATE)?1:0;
            add_char_params.char_props.notify         = (ch->perm&BLE_PERM_NOTIFY)?1:0;
            add_char_params.char_props.read           = (ch->perm&BLE_PERM_READ)?1:0;
            add_char_params.char_props.write_wo_resp  = (ch->perm&BLE_PERM_WRITE_WO)?1:0;
            add_char_params.char_props.write          = (ch->perm&BLE_PERM_WRITE)?1:0;
            add_char_params.char_props.auth_signed_wr = (ch->perm&BLE_PERM_WRITE_SIGNED)?1:0;

            add_char_params.cccd_write_access = SEC_OPEN;//p_bas_init->bl_cccd_wr_sec;
            add_char_params.read_access       = SEC_OPEN;//p_bas_init->bl_rd_sec;
            add_char_params.write_access       = SEC_OPEN;//p_bas_init->bl_rd_sec;

            err = characteristic_add(srv->handle, &add_char_params, &ch->handles);
            SDH_ERROR_CHECK(err,"failed adding characteristic");
        }
    }
    return 0;
}

C_NATIVE(_ble_start) {
    C_NATIVE_UNWARN();
    ret_code_t err;
    if (!bleevt) {
        bleevt = vosSemCreate(0);
    }
    gap_params_init();
    conn_params_init();
    err = nrf_ble_gatt_init(&vm_m_gatt, NULL);
    BLE_ERROR_CHECK(err,"Failed gatt",ERR_RUNTIME_EXC);
    err = peer_manager_init();
    BLE_ERROR_CHECK(err,"Failed peer_manager",ERR_RUNTIME_EXC);
    /* err = pm_peers_delete(); */
    /* BLE_ERROR_CHECK(err, "Failed peers delete",ERR_RUNTIME_EXC); */
    err = services_init();
    BLE_ERROR_CHECK(err, "Failed services init",ERR_RUNTIME_EXC);
    err = scan_init();
    BLE_ERROR_CHECK(err, "Failed scan init",ERR_RUNTIME_EXC);
    return ERR_OK;
}


