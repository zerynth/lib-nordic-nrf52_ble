#define ZERYNTH_PRINTF
#include "zerynth.h"
#include "ble_service.h"

//redefine some NRF macro
#define APP_ERROR_CHECK(err_code) if(err_code!=NRF_SUCCESS) return VHAL_HARDWARE_STATUS_ERROR
#define NRF_LOG_INFO(...)
#define NRF_LOG_DEBUG(...)


//define compilation flags for header files
#define BLE_STACK_SUPPORT_REQD 1
#define SOFTDEVICE_PRESENT 1
#define NRF52  1
#define NRF_SD_BLE_API_VERSION 3

#define PEER_MANAGER_ENABLED 1
#define BLE_ADVERTISING_ENABLED 1

#define FDS_ENABLED 1
#define FDS_CHUNK_QUEUE_SIZE 8
#define FDS_OP_QUEUE_SIZE 4
#define FDS_VIRTUAL_PAGES 3
#define FDS_MAX_USERS 8
#define FDS_VIRTUAL_PAGE_SIZE 1024

#define FSTORAGE_ENABLED 1
#define FS_QUEUE_SIZE 4
#define FS_OP_MAX_RETRIES 3
#define FS_MAX_WRITE_SIZE_WORDS 1024


//include NRF headers
#include "ble_ifc.h"
#include "ble.h"
#include "nrf_sdm.h"
#include "softdevice_handler.h"
#include "peer_manager.h"
#include "fds.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_srv_common.h"
#include "ble_conn_params.h"
#include "app_timer.h"



//default parameters
#define CENTRAL_LINK_COUNT               0                                /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT            1                                /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT             /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */


#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

#define SEC_PARAM_BOND                   1                                /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                   0                                /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS               0                                /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE             /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                               /**< Maximum encryption key size. */


#define FIRST_CONN_PARAMS_UPDATE_DELAY   5000                             /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    30000                            /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_ADV_INTERVAL                 300                              /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_FEATURE_NOT_SUPPORTED        BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */
#define APP_TIMER_PRESCALER              0                                /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                /**< Size of timer operation queues. */

#define BLE_NUM_EVENTS 8

static uint8_t *name;
static uint32_t namelen;
static uint32_t security;
static uint32_t level;
static uint32_t appearance;
static uint32_t min_conn;
static uint32_t max_conn;
static uint32_t latency;
static uint32_t conn_sup;
static uint32_t advertising_interval=APP_ADV_INTERVAL;
static uint32_t advertising_timeout=0;
static uint16_t  m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */
static VSemaphore blesem=NULL, bleevt=NULL;
static BLEService *services;
static uint32_t nserv;
static ble_uuid_t *m_adv_uuids;
static BLEEvent _events[BLE_NUM_EVENTS];
static uint16_t _event_head=0, _event_num=0;
static uint8_t advert_data[31];
static uint32_t advert_data_len=0;
static uint32_t list_uuids=1;


static uint32_t ble_new_event_handler(void)
{
    // The returned value may be safely ignored, if error is returned it only means that
    // the semaphore is already given (raised).
    vosSemSignalIsr(blesem);
    return NRF_SUCCESS;
}


#define NOTIFICATION_ENABLED(data) ((uint16_decode(data) & BLE_GATT_HVX_NOTIFICATION) ? 1:0)
#define INDICATION_ENABLED(data) ((uint16_decode(data) & BLE_GATT_HVX_INDICATION) ? 1:0)


static void _set_event(uint16_t type,uint16_t service, uint16_t characteristic, uint16_t status){
    vosSysLock();
    if (_event_num<BLE_NUM_EVENTS){
        _events[_event_head].type=type;
        _events[_event_head].service=service;
        _events[_event_head].characteristic=characteristic;
        _events[_event_head].status=status;
        _event_head=(_event_head+1)%BLE_NUM_EVENTS;
        _event_num++;
        vosSemSignalIsr(bleevt);
    }
    vosSysUnlock();
}


static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code = NRF_SUCCESS;

    //printf("BLE GEN %i\n",p_ble_evt->header.evt_id);
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            //printf("Disconnected.\r\n");

            //err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            //APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            _set_event(0,0,0,1); //type 0, status 1
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_CONNECTED:
            //err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            //APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            {
                int i,j;
                for (i=0;i<nserv;i++){
                    BLEService *service = &services[i];
                    for (j=0;j<service->nchs;j++){
                        BLEChar *bc = &service->chs[j];
                        bc->cccd=0; //reset cccd on connection
                    }
                }
            }
            _set_event(0,0,0,0); //type 0, status 0
            //printf("Connected. %i\r\n",m_conn_handle);
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            //printf("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            //APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            //printf("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            //APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            //APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST

        default:
            // No implementation needed.
            break;
    }
}

static void ble_on_service_evt(ble_evt_t * p_ble_evt){
    int i,j;
    //printf("BLE EVT %i %i %i %i\n",m_conn_handle,BLE_CONN_HANDLE_INVALID,p_ble_evt->header.evt_id,BLE_GATTS_EVT_WRITE);
    if (m_conn_handle==BLE_CONN_HANDLE_INVALID) return;
    for (i=0;i<nserv;i++){
        BLEService *service = &services[i];
        for (j=0;j<service->nchs;j++){
            BLEChar *bc = &service->chs[j];
            if(p_ble_evt->header.evt_id==BLE_GATTS_EVT_WRITE){
                ble_gatts_evt_write_t *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
                if (p_evt_write->handle == bc->handles.cccd_handle){
                    //cccd write
                    if (p_evt_write->len == 2){
                        uint16_t cccd = uint16_decode(p_evt_write->data);
                        bc->cccd ^= (BLE_PERM_NOTIFY * NOTIFICATION_ENABLED(p_evt_write->data)) | (BLE_PERM_INDICATE * INDICATION_ENABLED(p_evt_write->data));
                        //printf("cccd write %i %i %x\n",p_evt_write->handle,bc->cccd,cccd);
                        _set_event(1,service->uuid,bc->uuid,bc->cccd); //GATT EVENT, NOTIFY status
                    }
                } else if(p_evt_write->handle == bc->handles.value_handle){
                    //value write
                    //printf("value write\n");
                    _set_event(1,service->uuid,bc->uuid,BLE_PERM_WRITE); //GATT EVENT, WRITE
                } else if(p_evt_write->handle == bc->handles.sccd_handle){
                    //sccd write
                    //printf("sccd write\n");
                }
            } 
        }
    }
}

static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    /** The Connection state module has to be fed BLE events in order to function correctly
     * Remember to call ble_conn_state_on_ble_evt before calling any ble_conns_state_* functions. */
    //printf("EVT %i %i\n",p_ble_evt->header.evt_id,p_ble_evt->header.evt_len);
    ble_conn_state_on_ble_evt(p_ble_evt);
    pm_on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    //bsp_btn_ble_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    ble_on_service_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    // Dispatch the system event to the fstorage module, where it will be
    // dispatched to the Flash Data Storage (FDS) module.
    fs_sys_event_handler(sys_evt);
    
    // Dispatch to the Advertising module last, since it will check if there are any
    // pending flash operations in fstorage. Let fstorage process system events first,
    // so that it can report correctly to the Advertising module.
    ble_advertising_on_sys_evt(sys_evt);
    
}



static int pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    //printf("PM_EVT %i\n",p_evt->evt_id);
    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            //printf("Connected to a previously bonded device.\r\n");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Connection secured. Role: %d. conn_handle: %d, Procedure: %d\r\n",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            ble_advertising_start(BLE_ADV_MODE_FAST);
        } break;

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        {
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}


static int peer_manager_init(int erase_bonds)
{
    ble_gap_sec_params_t sec_param;
    uint32_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    if (erase_bonds)
    {
        err_code = pm_peers_delete();
        APP_ERROR_CHECK(err_code);
    }

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}




static int gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    //TODO: add security
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)name,
                                          namelen);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(appearance);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));


    gap_conn_params.min_conn_interval = (min_conn*1000)/1250;
    gap_conn_params.max_conn_interval = (max_conn*1000)/1250;
    gap_conn_params.slave_latency     = latency;
    gap_conn_params.conn_sup_timeout  = (conn_sup*1000)/10000;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
    return 0;
}



static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {

        case BLE_ADV_EVT_SLOW:

        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast Adverstising\r\n");
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            //APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            //sleep_mode_enter();
            break;

        default:
            break;
    }
}


static int advertising_init(void)
{
    uint32_t               err_code;
    ble_advdata_t          advdata;
    ble_adv_modes_config_t options;

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    if (list_uuids){
        advdata.name_type               = BLE_ADVDATA_FULL_NAME;
        advdata.include_appearance      = true;
        advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
        advdata.uuids_complete.uuid_cnt = list_uuids;//sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
        advdata.uuids_complete.p_uuids  = m_adv_uuids;
    } else {

    }

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = advertising_interval*1000/625;
    options.ble_adv_fast_timeout  = advertising_timeout;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
    return 0;
}

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        //APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    //APP_ERROR_HANDLER(nrf_error);
}


static int conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}



/********************* natives */

C_NATIVE(_ble_init) {
    C_NATIVE_UNWARN();
    uint32_t err_code;
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, ble_new_event_handler);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Check the ram settings against the used number of links
    //CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
    
    //peer_manager_init(0);
    //gaps_params_init();
    return ERR_OK;
}




C_NATIVE(_ble_set_gap) {
    C_NATIVE_UNWARN();
    *res = MAKE_NONE();
    if (parse_py_args("siiiiiii", nargs, args, &name, &namelen,&security,&level,&appearance,&min_conn,&max_conn,&latency,&conn_sup) != 8)
        return ERR_TYPE_EXC;
    //make a copy of name
    uint8_t *bname = gc_malloc(namelen);
    memcpy(bname,name,namelen);
    name=bname;
    //printf("Name is %c %c %c %c %i\n",name[0],name[1],name[2],name[3],namelen);
    return ERR_OK;
}


C_NATIVE(_ble_set_services) {
    C_NATIVE_UNWARN();
    PList *slist = args[0];
    nserv = PSEQUENCE_ELEMENTS(slist);
    m_adv_uuids = gc_malloc(sizeof(ble_uuid_t)*nserv);
    int i,j;
    uint16_t uuid_type = BLE_UUID_TYPE_BLE;
    services = gc_malloc(sizeof(BLEService)*nserv);
    *res  = MAKE_NONE();
    for(i=0;i<nserv;i++){
        BLEService *srv = &services[i];
        PTuple *psrv = (PTuple*)PLIST_ITEM(slist,i); //tuple of uuid, characteristics
        PList *chs = PTUPLE_ITEM(psrv,1); //get characteristics
        PObject *base_uuid = PTUPLE_ITEM(psrv,2);
        //printf("base_uuid %x, *res %x\n",base_uuid,*res);
        if(base_uuid!=*res){
            //extended uuid!
            ble_uuid128_t buuid;
            memcpy(buuid.uuid128,PSEQUENCE_BYTES(base_uuid),16);
            sd_ble_uuid_vs_add(&buuid, &uuid_type);
        }
        //printf("uuid_type %i\n",uuid_type);
        srv->uuid = (uint16_t)PSMALLINT_VALUE( PTUPLE_ITEM(psrv,0));
        srv->uuid_type = uuid_type;
        m_adv_uuids[i].uuid = srv->uuid;
        m_adv_uuids[i].type = uuid_type;
        srv->nchs = PSEQUENCE_ELEMENTS(chs);
        //printf("service %x %x %i\n",srv->uuid,chs,srv->nchs);
        //vosThSleep(TIME_U(500,MILLIS));
        srv->chs = gc_malloc(sizeof(BLEChar)*srv->nchs);
        for(j=0;j<srv->nchs;j++){
            BLEChar *sch = &srv->chs[j];
            PTuple *ch = PLIST_ITEM(chs,j); //take jth char
            sch->uuid = (uint16_t)PSMALLINT_VALUE(PTUPLE_ITEM(ch,0)); //uuid
            sch->permission = (uint16_t)PSMALLINT_VALUE(PTUPLE_ITEM(ch,2)); //permission
            
            PString *desc = (PString*)PTUPLE_ITEM(ch,1);
            sch->dlen = PSEQUENCE_ELEMENTS(desc);
            if(sch->dlen){
                sch->descriptor = gc_malloc(sch->dlen);
                memcpy(sch->descriptor,PSEQUENCE_BYTES(desc),sch->dlen);
            } else sch->descriptor=NULL;
            
            sch->ilen = (uint16_t)PSMALLINT_VALUE(PTUPLE_ITEM(ch,3)); //value size
        }
    }

    return ERR_OK;
}


C_NATIVE(_ble_get_event) {
    C_NATIVE_UNWARN();
    BLEEvent blee;
    int pos;
    RELEASE_GIL();
    vosSemWait(bleevt);
    vosSysLock();
    pos = (_event_head+BLE_NUM_EVENTS- _event_num)%BLE_NUM_EVENTS;
    _event_num--;
    memcpy(&blee,&_events[pos],sizeof(BLEEvent));
    vosSysUnlock();
    PTuple *tpl = ptuple_new(4,NULL);
    PTUPLE_SET_ITEM(tpl,0,PSMALLINT_NEW(blee.type));
    PTUPLE_SET_ITEM(tpl,1,PSMALLINT_NEW(blee.type));
    PTUPLE_SET_ITEM(tpl,2,PSMALLINT_NEW(blee.type));
    PTUPLE_SET_ITEM(tpl,3,PSMALLINT_NEW(blee.status));
    if (blee.type==1){
        //GATT EVENT
        PTUPLE_SET_ITEM(tpl,1,PSMALLINT_NEW(blee.service));
        PTUPLE_SET_ITEM(tpl,2,PSMALLINT_NEW(blee.characteristic));
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
    for(i=0;i<nserv;i++){
        BLEService *service = &services[i];
        if (service->uuid==serv_uuid){
            for(j=0;j<service->nchs;j++){
                BLEChar *bc = &service->chs[j];
                if (bc->uuid==uuid){
                    //found!
                    uint32_t err_code = NRF_SUCCESS;
                    ble_gatts_value_t gatts_value;
                    memset(&gatts_value, 0, sizeof(gatts_value));
                    PBytes *pb =pbytes_new(bc->ilen,NULL);

                    gatts_value.len     = bc->ilen;
                    gatts_value.offset  = 0;
                    gatts_value.p_value = PSEQUENCE_BYTES(pb);

                    err_code = sd_ble_gatts_value_get(m_conn_handle,
                                      bc->handles.value_handle,
                                      &gatts_value);
                    *res=pb;
                    return ERR_OK;
                }
            }
        }
    }

    return ERR_OK;
}

C_NATIVE(_ble_set_value) {
    C_NATIVE_UNWARN();
    uint32_t serv_uuid;
    uint32_t uuid;
    uint8_t* value;
    uint32_t valuelen;
    int i,j;
    *res = MAKE_NONE();
    if (parse_py_args("iis", nargs, args, &serv_uuid, &uuid,&value, &valuelen) != 3)
        return ERR_TYPE_EXC;
    for(i=0;i<nserv;i++){
        BLEService *service = &services[i];
        //printf("Service %i - %i\n",service->uuid,serv_uuid);
        if (service->uuid==serv_uuid){
            for(j=0;j<service->nchs;j++){
                BLEChar *bc = &service->chs[j];
                //printf("CH %i - %i\n",bc->uuid,uuid);
                if (bc->uuid==uuid){
                    //found!
                    uint32_t err_code = NRF_SUCCESS;
                    ble_gatts_value_t gatts_value;
                    memset(&gatts_value, 0, sizeof(gatts_value));
                    gatts_value.len     = valuelen;
                    gatts_value.offset  = 0;
                    gatts_value.p_value = value;
                    // Update database. 
                    err_code = sd_ble_gatts_value_set(m_conn_handle,
                                          bc->handles.value_handle,
                                          &gatts_value);
                    //if(err_code!=NRF_SUCCESS) return ERR_VALUE_EXC;
                    if((m_conn_handle!=BLE_CONN_HANDLE_INVALID) && (bc->cccd & BLE_PERM_NOTIFY)){
                        //printf("NOTIFY %i %i\n",m_conn_handle,bc->cccd);
                        //send notification
                        ble_gatts_hvx_params_t hvx_params;
                        memset(&hvx_params, 0, sizeof(hvx_params));

                        hvx_params.handle = bc->handles.value_handle;
                        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
                        hvx_params.offset = gatts_value.offset;
                        hvx_params.p_len  = &gatts_value.len;
                        hvx_params.p_data = gatts_value.p_value;

                        err_code = sd_ble_gatts_hvx(m_conn_handle, &hvx_params);
                        //printf("ERROR %i\n",err_code);
                        //if(err_code!=NRF_SUCCESS) return ERR_VALUE_EXC;
                    }
                    return ERR_OK;
                }
            }
        }
    }

    return ERR_INDEX_EXC;
}



C_NATIVE(_ble_set_advertising) {
    C_NATIVE_UNWARN();
    *res = MAKE_NONE();
    uint8_t *payload;
    uint32_t payloadlen;
    if (parse_py_args("iisi", nargs, args, &advertising_interval,&advertising_timeout,&payload,&payloadlen,&list_uuids) != 4)
        return ERR_TYPE_EXC;
    advert_data_len=payloadlen;
    if (payloadlen>31) advert_data_len=31;
    memcpy(advert_data,payload,advert_data_len);
    return ERR_OK;
}


C_NATIVE(_ble_start_advertising) {
    C_NATIVE_UNWARN();
    ble_advertising_start(BLE_ADV_MODE_FAST);
    return ERR_OK;
}

C_NATIVE(_ble_stop_advertising) {
    C_NATIVE_UNWARN();
    sd_ble_gap_adv_stop();
    return ERR_OK;
}


C_NATIVE(_ble_start_scanning) {
    C_NATIVE_UNWARN();
    return ERR_NOT_IMPLEMENTED_EXC;
}

C_NATIVE(_ble_stop_scanning) {
    C_NATIVE_UNWARN();
    return ERR_NOT_IMPLEMENTED_EXC;
}


void services_init(){
    if(!services) return;
    int i,j;
    uint32_t   err_code;
    ble_uuid_t ble_uuid;
    for (i=0;i<nserv;i++){
        BLEService *srv = &services[i];
         // Add service
        ble_uuid.type = srv->uuid_type;
        ble_uuid.uuid = srv->uuid;
        err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &srv->handle);
        for(j=0;j<srv->nchs;j++){
            //add characteristic
            BLEChar *ch = &srv->chs[j];
            ble_gatts_char_md_t char_md;
            ble_gatts_attr_md_t cccd_md;
            ble_gatts_attr_t    attr_char_value;
            ble_gatts_attr_md_t attr_md;

            memset(&cccd_md, 0, sizeof(cccd_md));

            BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
            BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
            cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

            memset(&char_md, 0, sizeof(char_md));

            char_md.char_props.broadcast = (ch->permission & BLE_PERM_BROADCAST) ? 1:0;
            char_md.char_props.notify = (ch->permission & BLE_PERM_NOTIFY) ? 1:0;
            char_md.char_props.indicate = (ch->permission & BLE_PERM_INDICATE) ? 1:0;
            char_md.char_props.read = (ch->permission & BLE_PERM_READ) ? 1:0;
            char_md.char_props.write = (ch->permission & BLE_PERM_WRITE) ? 1:0;
            char_md.char_props.write_wo_resp = (ch->permission & BLE_PERM_WRITE_WO) ? 1:0;
            char_md.char_props.auth_signed_wr = (ch->permission & BLE_PERM_WRITE_SIGNED) ? 1:0;
            char_md.p_char_user_desc  = ch->descriptor;
            char_md.char_user_desc_size  = ch->dlen;
            char_md.char_user_desc_max_size  = ch->dlen;
            char_md.p_char_pf         = NULL;
            char_md.p_user_desc_md    = NULL;
            char_md.p_cccd_md         = &cccd_md;
            char_md.p_sccd_md         = NULL;

            ble_uuid.uuid= ch->uuid; //type remains the same of service

            memset(&attr_md, 0, sizeof(attr_md));


            BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
            BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
            if (ch->permission & BLE_PERM_READ) BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
            if ((ch->permission & BLE_PERM_WRITE)||(ch->permission & BLE_PERM_WRITE_WO)) BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
            attr_md.vloc       = BLE_GATTS_VLOC_STACK;
            attr_md.rd_auth    = 0;
            attr_md.wr_auth    = 0;
            attr_md.vlen       = 1;

            memset(&attr_char_value, 0, sizeof(attr_char_value));

            uint8_t value=1;
            attr_char_value.p_uuid    = &ble_uuid;
            attr_char_value.p_attr_md = &attr_md;
            attr_char_value.init_len  = 1;
            attr_char_value.init_offs = 0;
            attr_char_value.max_len   = 1;
            attr_char_value.p_value   = &value;

            return sd_ble_gatts_characteristic_add(srv->handle,
                                                    &char_md,
                                                    &attr_char_value,
                                                    &ch->handles);
        }
    }
}

void _ble_thread(void *data){
    //init
    peer_manager_init(true);
    gap_params_init();
    advertising_init();
    services_init();
    conn_params_init();
    //printf("Started\n");
    while (1){
        vosSemWait(blesem);
        intern_softdevice_events_execute();
    }
}

C_NATIVE(_ble_start) {
    C_NATIVE_UNWARN();
    if (!blesem) {
        blesem = vosSemCreate(0);
        bleevt = vosSemCreate(0);
        VThread th = vosThCreate(512,VOS_PRIO_HIGH,_ble_thread,NULL,NULL);
        vosThResume(th);
    }
    return ERR_OK;
}

