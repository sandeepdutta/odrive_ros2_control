#include "odrive.hpp"
#include "odrive_endpoint.hpp"

using namespace std;

/**
 *
 * Odrive endpoint constructor
 * initialize USB library and local variables
 *
 */
odrive_endpoint::odrive_endpoint()
{
    if (libusb_init(&libusb_context_) != LIBUSB_SUCCESS) {
        ROS_ERROR("* Error initializing USB!");
    }
}

/**
 *
 * Odrive endpoint destructor
 * release USB library
 *
 */
odrive_endpoint::~odrive_endpoint()
{
    if (libusb_context_ != NULL) {
        libusb_exit(libusb_context_);
        libusb_context_ = NULL;
    }
}

/**
 *
 * Append short data to data buffer
 * @param buf data buffer
 * @param value data to append
 *
 */
void odrive_endpoint::appendShortToCommBuffer(commBuffer& buf, const short value)
{
    buf.push_back((value >> 0) & 0xFF);
    buf.push_back((value >> 8) & 0xFF);
}

/**
 *
 * Append int data to data buffer
 * @param buf data buffer
 * @param value data to append
 *
 */
void odrive_endpoint::appendIntToCommBuffer(commBuffer& buf, const int value)
{
    buf.push_back((value >> 0) & 0xFF);
    buf.push_back((value >> 8) & 0xFF);
    buf.push_back((value >> 16) & 0xFF);
    buf.push_back((value >> 24) & 0xFF);
}

/**
 *
 *  Decode odrive packet
 *  @param buf data buffer
 *  @param seq_no packet sequence number
 *  @param received_packet received buffer
 *  @return data buffer
 *
 */
commBuffer odrive_endpoint::decodeODrivePacket(commBuffer& buf,
    	short& seq_no, commBuffer& received_packet)
{
    commBuffer payload;

    memcpy(&seq_no, &buf[0], sizeof(short));
    seq_no &= 0x7fff;
    for (commBuffer::size_type i = 2; i < buf.size(); ++i) {
        payload.push_back(buf[i]);
    }
    return payload;
}

/**
 *
 * Read data buffer from Odrive harware
 * @param seq_no next sequence number
 * @param endpoint_id USB endpoint ID
 * @param response_size maximum data length to be read
 * @param read append request address
 * @param address desctination address
 * @param input data buffer to send
 * @return data buffer read
 *
 */
commBuffer odrive_endpoint::createODrivePacket(short seq_no, int endpoint_id,
                short response_size, bool read, int address, const commBuffer& input)
{
    commBuffer packet;
    short crc = 0;

    if ((endpoint_id & 0x7fff) == 0) {
        crc = ODRIVE_PROTOCOL_VERSION;
    }
    else {
        crc = (short)ODRIVE_DEFAULT_CRC_VALUE;
    }

    appendShortToCommBuffer(packet, seq_no);
    appendShortToCommBuffer(packet, endpoint_id);
    appendShortToCommBuffer(packet, response_size);
    if (read) {
        appendIntToCommBuffer(packet, address);
    }

    for (uint8_t b : input) {
        packet.push_back(b);
    }

    appendShortToCommBuffer(packet, crc);

    return packet;
}

/**
 *
 *  Read value from ODrive
 *  @param id odrive ID
 *  @param value Data read
 *  @return ODRIVE_OK on success
 *
 */
template<typename T>
int odrive_endpoint::getData(int id, T& value)
{
    commBuffer tx;
    commBuffer rx;
    int rx_size;

    int result = endpointRequest(id, rx,
                    rx_size, tx, 1 /* ACK */, sizeof(value));
    if (result != LIBUSB_SUCCESS) {
        return result;
    }

    memcpy(&value, &rx[0], sizeof(value));

    return LIBUSB_SUCCESS;
}


/**
 *
 *  Request function to ODrive
 *  @param id odrive ID
 *  @return ODRIVE_OK on success
 *
 */
int odrive_endpoint::execFunc(int endpoint_id)
{
    commBuffer tx;
    commBuffer rx;
    int rx_length;
    int status;

    status = endpointRequest(endpoint_id, rx, rx_length, tx, 1, 0);
    if (status != LIBUSB_SUCCESS) {
        ROS_ERROR("* execFunc: Error in endpoint request (%d)!", endpoint_id);
    }
    return status;
}

/**
 *
 *  Write value to Odrive
 *  @param id odrive ID
 *  @param value Data to be written
 *  @return ODRIVE_OK on success
 *
 */
template<typename TT>
int odrive_endpoint::setData(int endpoint_id, const TT& value)
{
    commBuffer tx;
    commBuffer rx;
    int rx_length;

    for(int64_t i = 0; i < sizeof(value); i++){
       tx.push_back(((unsigned char*)&value)[i]);
    }

    return endpointRequest(endpoint_id, rx, rx_length, tx, 1, 0);
}

/**
 *
 * Request endpoint
 * @param handle USB device handler
 * @param endpoint_id odrive ID
 * @param received_payload receive buffer
 * @param received_length receive length
 * @param payload data read
 * @param ack request acknowledge
 * @param length data length
 * @param read send read address
 * @param address read address
 * @return LIBUSB_SUCCESS on success
 *
 */
int odrive_endpoint::endpointRequest(int endpoint_id, commBuffer& received_payload,
    	int& received_length, commBuffer payload,
    	bool ack, int length, bool read, int address)
{
    commBuffer send_buffer;
    commBuffer receive_buffer;
    unsigned char receive_bytes[ODRIVE_MAX_RESULT_LENGTH] = { 0 };
    int sent_bytes = 0;
    int received_bytes = 0;
    short received_seq_no = 0;

    ep_lock.lock();

    // Prepare sequence number
    if (ack) {
        endpoint_id |= 0x8000;
    }
    outbound_seq_no_ = (outbound_seq_no_ + 1) & 0x7fff;
    outbound_seq_no_ |= LIBUSB_ENDPOINT_IN;
    short seq_no = outbound_seq_no_;

    // Create request packet
    commBuffer packet = createODrivePacket(seq_no, endpoint_id, length, read, address, payload);

    // Transfer paket to target
    int result = libusb_bulk_transfer(odrive_handle_, ODRIVE_OUT_EP,
    	    packet.data(), packet.size(), &sent_bytes, ODRIVE_TIMEOUT);
    if (result != LIBUSB_SUCCESS) {
    ROS_ERROR("* Error in transfering data to USB!");
        ep_lock.unlock();
        return result;
    } else if (packet.size() != sent_bytes) {
        ROS_ERROR("* Error in transfering data to USB, not all data transferred!");
    }

    // Get responce
    if (ack) {
        result = libusb_bulk_transfer(odrive_handle_, ODRIVE_IN_EP,
    		receive_bytes, ODRIVE_MAX_BYTES_TO_RECEIVE,
    		&received_bytes, ODRIVE_TIMEOUT);
        if (result != LIBUSB_SUCCESS) {
            ROS_ERROR("* Error in reading data from USB!");
            ep_lock.unlock();
            return result;
        }

        // Push recevived data to buffer
        for (int i = 0; i < received_bytes; i++) {
            receive_buffer.push_back(receive_bytes[i]);
        }

        received_payload = decodeODrivePacket(receive_buffer, received_seq_no, receive_buffer);
        if (received_seq_no != seq_no) {
            ROS_ERROR("* Error Received data out of order");
        }
        received_length = received_payload.size();
    }

    ep_lock.unlock();

    return LIBUSB_SUCCESS;
}

/**
 *
 * Odrive endpoint init
 * enumerate ODrive hardware
 * @param serialNumber odrive serial number
 * @return ODRIVE_OK on success
 *
 */
int odrive_endpoint::init(uint64_t serialNumber)    	
{
    libusb_device ** usb_device_list;
    int ret = 1;

    ssize_t device_count = libusb_get_device_list(libusb_context_, &usb_device_list);
    if (device_count <= 0) {
        return device_count;
    }

    for (size_t i = 0; i < device_count; ++i) {
        libusb_device *device = usb_device_list[i];
        libusb_device_descriptor desc = {0};

        int result = libusb_get_device_descriptor(device, &desc);
        if (result != LIBUSB_SUCCESS) {
            ROS_ERROR("* Error getting device descriptor");
            continue;
        }
        /* Check USB devicei ID */
        if (desc.idVendor == ODRIVE_USB_VENDORID && desc.idProduct == ODRIVE_USB_PRODUCTID) {

            libusb_device_handle *device_handle;
            if (libusb_open(device, &device_handle) != LIBUSB_SUCCESS) {
                ROS_ERROR("* Error opeening USB device");
                continue;
             }

            struct libusb_config_descriptor *config;
            result = libusb_get_config_descriptor(device, 0, &config);
            int ifNumber = 2; //config->bNumInterfaces;

            if ((libusb_kernel_driver_active(device_handle, ifNumber) != LIBUSB_SUCCESS) &&
                    (libusb_detach_kernel_driver(device_handle, ifNumber) != LIBUSB_SUCCESS)) {
                ROS_ERROR("* Driver error");
                libusb_close(device_handle);
                continue;
            }

            if ((result = libusb_claim_interface(device_handle, ifNumber)) !=  LIBUSB_SUCCESS) {
                ROS_ERROR("* Error claiming device");
                libusb_close(device_handle);
                continue;
            } else {
                bool attached_to_handle = false;
                unsigned char buf[128];
 
 		result = libusb_get_string_descriptor_ascii(device_handle, desc.iSerialNumber, buf, 127);
                if (result <= 0) {
                    ROS_ERROR("* Error getting data");
                    result = libusb_release_interface(device_handle, ifNumber);
                    libusb_close(device_handle);
                    continue;
                } else {
                    std::stringstream stream;
                    stream << uppercase << std::hex << serialNumber;
                    std::string sn(stream.str());

                    if (sn.compare(0, strlen((const char*)buf), (const char*)buf) == 0 || serialNumber == 0) {
                        ROS_INFO("Device 0x%8.8lX Found", serialNumber);
                        odrive_handle_ = device_handle;
                        serial_number_ = serialNumber;
                        attached_to_handle = true;
                        ret = ODRIVE_OK;
                        break;
                    }    	
                }
                if (!attached_to_handle) {
                    result = libusb_release_interface(device_handle, ifNumber);
                    libusb_close(device_handle);
                }
            }
        }
    }

    libusb_free_device_list(usb_device_list, 1);

    return ret;
}

/**
 *
 * Odrive endpoint remove
 * close ODrive dvice
 *
 */
void odrive_endpoint::remove(void)
{
    if (odrive_handle_ != NULL) {
        libusb_release_interface(odrive_handle_, 2);
        libusb_close(odrive_handle_);
        odrive_handle_ = NULL;
    }
}

template int odrive_endpoint::getData(int, bool&);
template int odrive_endpoint::getData(int, short&);
template int odrive_endpoint::getData(int, int&);
template int odrive_endpoint::getData(int, float&);
template int odrive_endpoint::getData(int, uint8_t&);
template int odrive_endpoint::getData(int, uint16_t&);
template int odrive_endpoint::getData(int, uint32_t&);
template int odrive_endpoint::getData(int, uint64_t&);

template int odrive_endpoint::setData(int, const bool&);
template int odrive_endpoint::setData(int, const short&);
template int odrive_endpoint::setData(int, const int&);
template int odrive_endpoint::setData(int, const float&);
template int odrive_endpoint::setData(int, const uint8_t&);
template int odrive_endpoint::setData(int, const uint16_t&);
template int odrive_endpoint::setData(int, const uint32_t&);
template int odrive_endpoint::setData(int, const uint64_t&);



using namespace std;

/**
 *
 *  update odrive config
 *  Configure parameter on odrive hardware
 *  @param endpoint odrive enumarated endpoint
 *  @param odrive_json target json
 *  @param config_json json including configuration parameters
 *  @return ODRIVE_OK on success
 *
 */
int updateTargetConfig(odrive_endpoint *endpoint, Json::Value odrive_json, string config_file)
{
    ifstream cfg;
    string line, json;
    cfg.open (config_file, ios::in);
    cout << config_file;
    if (cfg.is_open()) {
        while (getline(cfg, line)) {
            json.append(line);
        }
        cfg.close();
        Json::Reader reader;
        Json::Value config_json;
        bool res = reader.parse(json, config_json);
        if (!res) {
            ROS_ERROR("* Error parsing %s json!", config_file.c_str());
            return ODRIVE_ERROR;
        }
        else {
            if (setChannelConfig(endpoint, odrive_json, config_json, false) != ODRIVE_OK) {
                ROS_ERROR("* Error setting configuration!");
                return ODRIVE_ERROR;
            }
        }
        if (calibrateAxis0(endpoint, odrive_json) != ODRIVE_OK) {
            ROS_ERROR("* Error calibrating axis 0!");
            return ODRIVE_ERROR;
        }
        if (calibrateAxis1(endpoint, odrive_json) != ODRIVE_OK) {
            ROS_ERROR("* Error calibrating axis 1!");
            return ODRIVE_ERROR;
        }
        if (execOdriveFunc(endpoint, odrive_json, "save_configuration") != ODRIVE_OK) {
            ROS_ERROR("* Error saving configuration!");
            return ODRIVE_ERROR;
        }
    }
    else {
        ROS_ERROR("* Error opening configuration file!");
    return ODRIVE_ERROR;
    }
    return ODRIVE_OK;
}

/**
 *
 *  Set odrive config
 *  Configure parameter on odrive hardware
 *  @param endpoint odrive enumarated endpoint
 *  @param odrive_json target json
 *  @param config_json json including configuration parameters
 *  @return ODRIVE_OK on success
 *
 */
int setChannelConfig(odrive_endpoint *endpoint, Json::Value odrive_json, Json::Value config_json,
    	bool save_config = 0)
{
    int ret = ODRIVE_OK;

    for (int i = 0 ; i < config_json.size() ; i++) {
    string name = config_json[i]["name"].asString();
    string type = config_json[i]["type"].asString();

    ROS_INFO("Setting %s config value", name.c_str());

    if (!type.compare("float")) {
            float val = config_json[i]["value"].asFloat();
            writeOdriveData(endpoint, odrive_json, name, val);
    }
        else if (!type.compare("uint8")) {
            uint8_t val = config_json[i]["value"].asUInt();
            writeOdriveData(endpoint, odrive_json, name, val);
    }
        else if (!type.compare("uint16")) {
            uint16_t val = config_json[i]["value"].asUInt();
            writeOdriveData(endpoint, odrive_json, name, val);
    }
        else if (!type.compare("uint32")) {
            uint32_t val = config_json[i]["value"].asUInt();
            writeOdriveData(endpoint, odrive_json, name, val);
    }
        else if (!type.compare("uint64")) {
            uint64_t val = config_json[i]["value"].asUInt64();
            writeOdriveData(endpoint, odrive_json, name, val);
    }
        else if (!type.compare("int32")) {
            int val = config_json[i]["value"].asInt();
            writeOdriveData(endpoint, odrive_json, name, val);
    }
        else if (!type.compare("int16")) {
            short val = config_json[i]["value"].asInt();
            writeOdriveData(endpoint, odrive_json, name, val);
    }
        else if (!type.compare("bool")) {
            bool val = config_json[i]["value"].asBool();
            writeOdriveData(endpoint, odrive_json, name, val);
    }
        else {
            ROS_ERROR("* Error: invalid type for %s", name.c_str());
            return ODRIVE_ERROR;
        }
    }

    // Save configuration
    if (save_config) {
        ret = execOdriveFunc(endpoint, odrive_json, "save_configuration");
    }

    return ret;
}

/**
 *
 *  Calibrate odrive axis 0
 *  Run calibration for axis 0
 *  @param endpoint odrive enumarated endpoint
 *  @param odrive_json target json
 *  @return ODRIVE_OK on success
 *
 */
int calibrateAxis0(odrive_endpoint *endpoint, Json::Value odrive_json)
{
    float fval;
    uint32_t u32val;
    bool bval;

    u32val = AXIS_STATE_MOTOR_CALIBRATION;
    writeOdriveData(endpoint, odrive_json, string("axis0.requested_state"), u32val);
    sleep(10);//ros::Duration(10.0).sleep();
    bval = true;
    writeOdriveData(endpoint, odrive_json, string("axis0.motor.config.pre_calibrated"), bval);
    u32val = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
    writeOdriveData(endpoint, odrive_json, string("axis0.requested_state"), u32val);
    sleep(10);//ros::Duration(10.0).sleep();
    bval = true;
    writeOdriveData(endpoint, odrive_json, string("axis0.encoder.config.pre_calibrated"), bval);

    fval = 3.0;
    writeOdriveData(endpoint, odrive_json, string("axis0.config.watchdog_timeout"), fval);

    execOdriveFunc(endpoint, odrive_json, string("save_configuration"));
    return 0;
}

/**
 *
 *  Calibrate odrive axis 1
 *  Run calibration for axis 1
 *  @param endpoint odrive enumarated endpoint
 *  @param odrive_json target json
 *  @return ODRIVE_OK on success
 *
 */
int calibrateAxis1(odrive_endpoint *endpoint, Json::Value odrive_json)
{
    float fval;
    uint32_t u32val;
    bool bval;

    u32val = AXIS_STATE_MOTOR_CALIBRATION;
    writeOdriveData(endpoint, odrive_json, string("axis1.requested_state"), u32val);
    sleep(10);//ros::Duration(10.0).sleep();
    bval = true;
    writeOdriveData(endpoint, odrive_json, string("axis1.motor.config.pre_calibrated"), bval);
    u32val = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
    writeOdriveData(endpoint, odrive_json, string("axis1.requested_state"), u32val);
    sleep(10);//ros::Duration(10.0).sleep();
    bval = true;
    writeOdriveData(endpoint, odrive_json, string("axis1.encoder.config.pre_calibrated"), bval);

    fval = 3.0;
    writeOdriveData(endpoint, odrive_json, string("axis1.config.watchdog_timeout"), fval);

    execOdriveFunc(endpoint, odrive_json, string("save_configuration"));
    return ODRIVE_OK;
}

/**
 *
 *  Read JSON file from target
 *  @param endpoint odrive enumarated endpoint
 *  @param odrive_json pointer to target json object
 *
 */
int getJson(odrive_endpoint *endpoint, Json::Value *odrive_json)
{

    commBuffer rx;
    commBuffer tx;
    int len;
    int address = 0;
    string json;

    do {
        endpoint->endpointRequest(0, rx, len, tx, true, 64, true, address);
        address = address + len;
        json.append((const char *)&rx[0], (size_t)len);
    } while (len > 0);

    Json::Reader reader;
    bool res = reader.parse(json, *odrive_json);
    if (!res) {
        ROS_ERROR("* Error parsing json!");
        return 1;
    }
    return 0;
}

/**
 *
 *  Scan for object name in target JSON
 *  @param odrive_json target json
 *  @param name object name to be found
 *  @param odo odrive object pointer including object parameters
 *  @return ODRIVE_OK on success
 *
 */
int getObjectByName(Json::Value odrive_json, std::string name, odrive_object *odo)
{
    int ret = -1;
    int i, pos;
    string token;
    Json::Value js;
    Json::Value js2 = odrive_json;

    while ((pos = name.find(".")) != std::string::npos) {
    js = js2;
        token = name.substr(0, pos);
        for (i = 0 ; i < js.size() ; i++) {
            if (!token.compare(js[i]["name"].asString())) {
    	if (!string("object").compare(js[i]["type"].asString())) {
                js2 = js[i]["members"];
    	}
    	else {
                    js2 = js[i];
    	}
    	break;
            }
        }
    name.erase(0, pos + 1);
    }

    for (i = 0 ; i < js2.size() ; i++) {
        if (!name.compare(js2[i]["name"].asString())) {
            odo->name = js2[i]["name"].asString();
            odo->id = js2[i]["id"].asInt();
            odo->type = js2[i]["type"].asString();
            odo->access = js2[i]["access"].asString();
        ret = 0;
        break;
        }
    }

    if (ret) {
        ROS_ERROR("* %s not found!", name.c_str());
    }
    return ret;
}

/**
 *
 *  Read single value from target
 *  @param endpoint odrive enumarated endpoint
 *  @param odrive_json target json
 *  @param value return value
 *  @return ODRIVE_OK on success
 *
 */
template<typename TT>
int readOdriveData(odrive_endpoint *endpoint, Json::Value odrive_json,
    	std::string object, TT &value)
{
    int ret;
    odrive_object odo;

    ret = getObjectByName(odrive_json, object, &odo);
    if (ret) {
        ROS_ERROR("* Error getting ID for %s", object.c_str());
    return ret;
    }

    if (odo.access.find("r") == string::npos) {
        ROS_ERROR("* Error: invalid read access for %s", object.c_str());
        return ret;
    }

    if (!odo.type.compare("float")) {
    if (sizeof(value) != sizeof(float)) {
            ROS_ERROR("* Error value for %s is not float", object.c_str());
            return ODRIVE_ERROR;
    }
    }
    else if (!odo.type.compare("uint8")) {
        if (sizeof(value) != sizeof(uint8_t)) {
            ROS_ERROR("* Error value for %s is not uint8_t", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("uint16")) {
        if (sizeof(value) != sizeof(uint16_t)) {
            ROS_ERROR("* Error value for %s is not uint16_t", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("uint32")) {
        if (sizeof(value) != sizeof(uint32_t)) {
            ROS_ERROR("* Error value for %s is not uint32_t", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("uint64")) {
        if (sizeof(value) != sizeof(uint64_t)) {
            ROS_ERROR("* Error value for %s is not uint64_t", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("int32")) {
        if (sizeof(value) != sizeof(int)) {
            ROS_ERROR("* Error value for %s is not int", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("int16")) {
        if (sizeof(value) != sizeof(short)) {
            ROS_ERROR("* Error value for %s is not short", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("bool")) {
        if (sizeof(value) != sizeof(bool)) {
            ROS_ERROR("* Error value for %s is not bool", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else {
        ROS_ERROR("* Error: invalid type for %s", object.c_str());
        return ODRIVE_ERROR;
    }

    ret = endpoint->getData(odo.id, value);

    return ret;
}

/**
 *
 *  Write single value to target
 *  @param endpoint odrive enumarated endpoint
 *  @param odrive_json target json
 *  @param value value to be written
 *  @return ODRIVE_OK on success
 *
 */
template<typename T>
int writeOdriveData(odrive_endpoint *endpoint, Json::Value odrive_json,
                std::string object, T &value)
{
    int ret;
    odrive_object odo;

    ret = getObjectByName(odrive_json, object, &odo);
    if (ret) {
        ROS_ERROR("* Error: getting ID for %s", object.c_str());
        return ODRIVE_ERROR;
    }

    if (odo.access.find("w") == string::npos) {
        ROS_ERROR("* Error: invalid write access for %s", object.c_str());
        return ODRIVE_ERROR;
    }

    if (!odo.type.compare("float")) {
        if (sizeof(value) != sizeof(float)) {
            ROS_ERROR("* Error value for %s is not float", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("uint8")) {
        if (sizeof(value) != sizeof(uint8_t)) {
            ROS_ERROR("* Error value for %s is not uint8_t", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("uint16")) {
        if (sizeof(value) != sizeof(uint16_t)) {
            ROS_ERROR("* Error value for %s is not uint16_t", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("uint32")) {
        if (sizeof(value) != sizeof(uint32_t)) {
            ROS_ERROR("* Error value for %s is not uint32_t", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("uint64")) {
        if (sizeof(value) != sizeof(uint64_t)) {
            ROS_ERROR("* Error value for %s is not uint64_t", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("int32")) {
        if (sizeof(value) != sizeof(int)) {
            ROS_ERROR("* Error value for %s is not int", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("int16")) {
        if (sizeof(value) != sizeof(short)) {
            ROS_ERROR("* Error value for %s is not short", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("bool")) {
        if (sizeof(value) != sizeof(bool)) {
            ROS_ERROR("* Error value for %s is not bool", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else {
        ROS_ERROR("* Error: invalid type for %s", object.c_str());
        return ODRIVE_ERROR;
    }

    ret = endpoint->setData(odo.id, value);

    return ret;
}

/**
 *
 *  Exec target function
 *  @param endpoint odrive enumarated endpoint
 *  @param odrive_json target json
 *  @param object name
 *  @return ODRIVE_OK on success
 *
 */
int execOdriveFunc(odrive_endpoint *endpoint, Json::Value odrive_json, std::string object)
{
    int ret;
    odrive_object odo;

    ret = getObjectByName(odrive_json, object, &odo);
    if (ret) {
        ROS_ERROR("* Error getting ID for %s", object.c_str());
        return ret;
    }

    if (odo.type.compare("function")) {
        ROS_ERROR("* Error invalid type not a function %s",object.c_str());
        return ret;
    }

    ret = endpoint->execFunc(odo.id);
    if (ret != LIBUSB_SUCCESS) {
        ROS_ERROR("* Error executing %s function", object.c_str());
    }
    return ret;
}

template int writeOdriveData(odrive_endpoint *, Json::Value, std::string, uint8_t &);
template int writeOdriveData(odrive_endpoint *, Json::Value, std::string, uint16_t &);
template int writeOdriveData(odrive_endpoint *, Json::Value, std::string, uint32_t &);
template int writeOdriveData(odrive_endpoint *, Json::Value, std::string, uint64_t &);
template int writeOdriveData(odrive_endpoint *, Json::Value, std::string, int &);
template int writeOdriveData(odrive_endpoint *, Json::Value, std::string, short &);
template int writeOdriveData(odrive_endpoint *, Json::Value, std::string, float &);
template int writeOdriveData(odrive_endpoint *, Json::Value, std::string, bool &);

template int readOdriveData(odrive_endpoint *, Json::Value, std::string, uint8_t &);
template int readOdriveData(odrive_endpoint *, Json::Value, std::string, uint16_t &);
template int readOdriveData(odrive_endpoint *, Json::Value, std::string, uint32_t &);
template int readOdriveData(odrive_endpoint *, Json::Value, std::string, uint64_t &);
template int readOdriveData(odrive_endpoint *, Json::Value, std::string, int &);
template int readOdriveData(odrive_endpoint *, Json::Value, std::string, short &);
template int readOdriveData(odrive_endpoint *, Json::Value, std::string, float &);
template int readOdriveData(odrive_endpoint *, Json::Value, std::string, bool &);

