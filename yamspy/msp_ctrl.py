import time

JUMBO_FRAME_SIZE_LIMIT = 255


dataHandler_init = {
    'msp_version':                1,
    'state':                      0,
    'message_direction':          -1,
    'code':                       -1,
    'dataView':                   [],
    'message_length_expected':    0,
    'message_length_received':    0,
    'message_buffer':             [],
    'message_buffer_uint8_view':  [],
    'message_checksum':           0,
    'messageIsJumboFrame':        False,
    'crcError':                   False,
    'packet_error':               0,
    'unsupported':                0,
    'pending':                    0,

    'last_received_timestamp':   None
}


read_buffer = b''
def _read(local_read):
    def read(buffer=None):
        global read_buffer
        if buffer:
            read_buffer = buffer

        if len(read_buffer)==0:
            return local_read() # read (try) everything in the serial/socket buffer
        else:
            output = b'' + read_buffer
            read_buffer = b''
            return output

    return read


def receive_msg(local_read, logging, dataHandler=None, output_raw_bytes=False, delete_buffer=False):
    """Receive an MSP message from the serial port
    Based on betaflight-configurator (https://git.io/fjRAz)

    Returns
    -------
    dict
        dataHandler with the received data pre-parsed
    """
    local_read = _read(local_read)
    if dataHandler is None:
        dataHandler = dataHandler_init.copy()
    else:
        dataHandler['pending'] = 0

    if output_raw_bytes:
        raw_bytes = b''

    di = 0
    while True:
        try:
            if di == 0:
                if delete_buffer:
                    received_bytes = memoryview(local_read(buffer=b'')) # it will 'fresh' read everything
                else:
                    received_bytes = memoryview(local_read()) # it will read everything from the buffer
                if received_bytes:
                    dataHandler['last_received_timestamp'] = time.time()
                    data = received_bytes[di]
                    if output_raw_bytes:
                        raw_bytes += received_bytes[di:di+1]
                else:
                    dataHandler['packet_error'] = 1
                    dataHandler['code'] = -1
                    break
            else:
                data = received_bytes[di]
                if output_raw_bytes:
                    raw_bytes += received_bytes[di:di+1]

            di += 1
            logging.debug(f"State: {dataHandler['state']} - byte received (at {dataHandler['last_received_timestamp']}): {data}")
        except IndexError:
            logging.debug('IndexError detected on state: {}'.format(dataHandler['state']))
            dataHandler['pending'] = 1
            break

        # it will always fall in the first state by default
        if dataHandler['state'] == 0: # sync char 1
            if (data == 36): # $ - a new MSP message begins with $
                dataHandler['state'] = 1

        elif dataHandler['state'] == 1: # sync char 2
            if (data == 77): # M - followed by an M => MSP V1
                dataHandler['msp_version'] = 1
                dataHandler['state'] = 2
            elif (data == 88): # X => MSP V2
                dataHandler['msp_version'] = 2
                dataHandler['state'] = 2
            else: # something went wrong, no M received...
                logging.debug('Something went wrong, no M or X received.')
                dataHandler['packet_error'] = 1
                dataHandler['code'] = -2
                break # sends it to the error state

        elif dataHandler['state'] == 2: # direction (should be >)
            dataHandler['unsupported'] = 0
            if (data == 33): # !
                # FC reports unsupported message error
                logging.debug('FC reports unsupported message error.')
                dataHandler['unsupported'] = 1
                dataHandler['packet_error'] = 1
                dataHandler['code'] = -3
                break # sends it to the error state
            else:
                if (data == 62): # > FC to PC
                    dataHandler['message_direction'] = 1
                elif (data == 60): # < PC to FC
                    dataHandler['message_direction'] = 0
                    
                if dataHandler['msp_version'] == 1:
                    dataHandler['state'] = 3

                elif dataHandler['msp_version'] == 2:
                    dataHandler['state'] = 2.1


        elif dataHandler['state'] == 2.1: # MSP V2: flag (ignored)
            dataHandler['flags'] = data # 4th byte 
            dataHandler['state'] = 2.2

        elif dataHandler['state'] == 2.2: # MSP V2: code LOW
            dataHandler['code'] = data
            dataHandler['state'] = 2.3

        elif dataHandler['state'] == 2.3: # MSP V2: code HIGH
            dataHandler['code'] |= data << 8
            dataHandler['state'] = 3.1

        elif dataHandler['state'] == 3:
            dataHandler['message_length_expected'] = data # 4th byte
            if dataHandler['message_length_expected'] == JUMBO_FRAME_SIZE_LIMIT:
                logging.debug("JumboFrame received.")
                dataHandler['messageIsJumboFrame'] = True

            # start the checksum procedure
            dataHandler['message_checksum'] = data
            dataHandler['state'] = 4

        elif dataHandler['state'] == 3.1: # MSP V2: msg length LOW
            dataHandler['message_length_expected'] = data
            dataHandler['state'] = 3.2

        elif dataHandler['state'] == 3.2: # MSP V2: msg length HIGH
            dataHandler['message_length_expected'] |= data << 8
            # setup buffer according to the message_length_expected
            dataHandler['message_buffer_uint8_view'] = dataHandler['message_buffer'] # keep same names from betaflight-configurator code
            if dataHandler['message_length_expected'] > 0:
                dataHandler['state'] = 7
            else:
                dataHandler['state'] = 9

        elif dataHandler['state'] == 4:
            dataHandler['code'] = data
            dataHandler['message_checksum'] ^= data

            if dataHandler['message_length_expected'] > 0:
                # process payload
                if dataHandler['messageIsJumboFrame']:
                    dataHandler['state'] = 5
                else:
                    dataHandler['state'] = 7
            else:
                # no payload
                dataHandler['state'] = 9

        elif dataHandler['state'] == 5:
            # this is a JumboFrame
            dataHandler['message_length_expected'] = data

            dataHandler['message_checksum'] ^= data

            dataHandler['state'] = 6

        elif dataHandler['state'] == 6:
            # calculates the JumboFrame size
            dataHandler['message_length_expected'] +=  256 * data
            logging.debug("JumboFrame message_length_expected: {}".format(dataHandler['message_length_expected']))
            # There's no way to check for transmission errors here...
            # In the worst scenario, it will try to read 255 + 256*255 = 65535 bytes
            dataHandler['message_checksum'] ^= data
            dataHandler['state'] = 7

        elif dataHandler['state'] == 7:
            # setup buffer according to the message_length_expected
            dataHandler['message_buffer'] = bytearray(dataHandler['message_length_expected'])
            dataHandler['message_buffer_uint8_view'] = dataHandler['message_buffer'] # keep same names from betaflight-configurator code

            # payload
            dataHandler['message_buffer_uint8_view'][dataHandler['message_length_received']] = data
            dataHandler['message_checksum'] ^= data
            dataHandler['message_length_received'] += 1

            if dataHandler['message_length_received'] == dataHandler['message_length_expected']:
                dataHandler['state'] = 9
            else:
                dataHandler['state'] = 8

        elif dataHandler['state'] == 8:
            # payload
            dataHandler['message_buffer_uint8_view'][dataHandler['message_length_received']] = data
            dataHandler['message_checksum'] ^= data
            dataHandler['message_length_received'] += 1

            if dataHandler['message_length_received'] == dataHandler['message_length_expected']:
                dataHandler['state'] = 9

        elif dataHandler['state'] == 9:
                if dataHandler['msp_version'] == 1:
                    if dataHandler['message_checksum'] == data:
                        # checksum is correct, message received, store dataview
                        logging.debug("Message received (length {1}) - Code {0}".format(dataHandler['code'], dataHandler['message_length_received']))
                        dataHandler['dataView'] = dataHandler['message_buffer'] # keep same names from betaflight-configurator code
                        break
                    else:
                        # wrong checksum
                        logging.debug('Code: {0} - crc failed (received {1}, calculated {2})'.format(dataHandler['code'], 
                                                                                                    data,
                                                                                                    dataHandler['message_checksum']))
                        dataHandler['crcError'] = True
                        dataHandler['packet_error'] = 1 # sends it to the error state
                        break 
                elif dataHandler['msp_version'] == 2:
                    dataHandler['message_checksum'] = 0
                    dataHandler['message_checksum'] = _crc8_dvb_s2(dataHandler['message_checksum'], 0) # flag
                    dataHandler['message_checksum'] = _crc8_dvb_s2(dataHandler['message_checksum'], dataHandler['code'] & 0xFF) # code LOW
                    dataHandler['message_checksum'] = _crc8_dvb_s2(dataHandler['message_checksum'], (dataHandler['code'] & 0xFF00) >> 8) # code HIGH
                    dataHandler['message_checksum'] = _crc8_dvb_s2(dataHandler['message_checksum'], dataHandler['message_length_expected'] & 0xFF) #  HIGH
                    dataHandler['message_checksum'] = _crc8_dvb_s2(dataHandler['message_checksum'], (dataHandler['message_length_expected'] & 0xFF00) >> 8) #  HIGH
                    for si in range(dataHandler['message_length_received']):
                        dataHandler['message_checksum'] = _crc8_dvb_s2(dataHandler['message_checksum'], dataHandler['message_buffer'][si])
                    if dataHandler['message_checksum'] == data:
                        # checksum is correct, message received, store dataview
                        logging.debug("Message received (length {1}) - Code {0}".format(dataHandler['code'], dataHandler['message_length_received']))
                        dataHandler['dataView'] = dataHandler['message_buffer'] # keep same names from betaflight-configurator code
                        break
                    else:
                        # wrong checksum
                        logging.debug('Code: {0} - crc failed (received {1}, calculated {2})'.format(dataHandler['code'], 
                                                                                                    data,
                                                                                                    dataHandler['message_checksum']))
                        dataHandler['crcError'] = True
                        dataHandler['packet_error'] = 1 # sends it to the error state
                        break

    if dataHandler['packet_error'] == 1:
        # it means an error occurred
        logging.debug('Error detected on state: {}'.format(dataHandler['state']))
    
    if len(received_bytes[di:]):
        local_read(buffer=received_bytes[di:]) # regurgitates unread stuff :)

    if output_raw_bytes:
        return dataHandler, raw_bytes
    else:
        return dataHandler 


def prepare_RAW_msg(mspv, code, data=[]):
    """Send a RAW MSP message through the serial port
    Based on betaflight-configurator (https://git.io/fjRxz)

    Parameters
    ----------
    code : int
        MSP Code
    
    data: list or bytearray, optional
        Data to be sent (default is [])
        
    Returns
    -------
    int
        number of bytes of data actually written (including 6 bytes header)
    """

    res = -1

    # Always reserve 6 bytes for protocol overhead
    # $ + M + < + data_length + msg_code + data + msg_crc
    len_data = len(data)

    if len_data > 256: # this shouldn't be needed...
        mspv = 2

    if mspv==1: # MSP V1
        size = len_data + 6
        checksum = 0

        bufView = bytearray(size)

        bufView[0] = 36 #$
        bufView[1] = 77 #M
        bufView[2] = 60 #<
        bufView[3] = len_data
        bufView[4] = code

        checksum = bufView[3] ^ bufView[4]

        for i in range(len_data):
            bufView[i + 5] = data[i]
            checksum ^= bufView[i + 5]

        bufView[-1] = checksum

    elif mspv==2: # MSP V2
        size = len_data + 9
        checksum = 0
        bufView = bytearray(size)
        bufView[0] = 36 #$ 
        bufView[1] = 88 #X
        bufView[2] = 60 #<
        bufView[3] = 0 #flag: reserved, set to 0
        bufView[4] = code & 0xFF #code lower byte
        bufView[5] = (code & 0xFF00) >> 8 #code upper byte
        bufView[6] = len_data & 0xFF #len_data lower byte
        bufView[7] = (len_data & 0xFF00) >> 8 #len_data upper byte
        bufView[8:8+len_data] = data
        for si in range(3, size-1):
            checksum = _crc8_dvb_s2(checksum, bufView[si])
        bufView[-1] = checksum

    else:
        return []

    return bufView


# def _crc8_dvb_s2(crc, ch):
#     """CRC for MSPV2
#     *copied from inav-configurator
#     """
#     crc ^= ch
#     for _ in range(8):
#         if (crc & 0x80):
#             crc = ((crc << 1) & 0xFF) ^ 0xD5
#         else:
#             crc = (crc << 1) & 0xFF
#     return crc


# from https://github.com/fishpepper/openTCO/blob/master/crc8.h
crc8_table = [
    0x00, 0xd5, 0x7f, 0xaa, 0xfe, 0x2b, 0x81, 0x54, 0x29, 0xfc, 0x56, 0x83, 0xd7, 0x02, 0xa8, 0x7d,
    0x52, 0x87, 0x2d, 0xf8, 0xac, 0x79, 0xd3, 0x06, 0x7b, 0xae, 0x04, 0xd1, 0x85, 0x50, 0xfa, 0x2f,
    0xa4, 0x71, 0xdb, 0x0e, 0x5a, 0x8f, 0x25, 0xf0, 0x8d, 0x58, 0xf2, 0x27, 0x73, 0xa6, 0x0c, 0xd9,
    0xf6, 0x23, 0x89, 0x5c, 0x08, 0xdd, 0x77, 0xa2, 0xdf, 0x0a, 0xa0, 0x75, 0x21, 0xf4, 0x5e, 0x8b,
    0x9d, 0x48, 0xe2, 0x37, 0x63, 0xb6, 0x1c, 0xc9, 0xb4, 0x61, 0xcb, 0x1e, 0x4a, 0x9f, 0x35, 0xe0,
    0xcf, 0x1a, 0xb0, 0x65, 0x31, 0xe4, 0x4e, 0x9b, 0xe6, 0x33, 0x99, 0x4c, 0x18, 0xcd, 0x67, 0xb2,
    0x39, 0xec, 0x46, 0x93, 0xc7, 0x12, 0xb8, 0x6d, 0x10, 0xc5, 0x6f, 0xba, 0xee, 0x3b, 0x91, 0x44,
    0x6b, 0xbe, 0x14, 0xc1, 0x95, 0x40, 0xea, 0x3f, 0x42, 0x97, 0x3d, 0xe8, 0xbc, 0x69, 0xc3, 0x16,
    0xef, 0x3a, 0x90, 0x45, 0x11, 0xc4, 0x6e, 0xbb, 0xc6, 0x13, 0xb9, 0x6c, 0x38, 0xed, 0x47, 0x92,
    0xbd, 0x68, 0xc2, 0x17, 0x43, 0x96, 0x3c, 0xe9, 0x94, 0x41, 0xeb, 0x3e, 0x6a, 0xbf, 0x15, 0xc0,
    0x4b, 0x9e, 0x34, 0xe1, 0xb5, 0x60, 0xca, 0x1f, 0x62, 0xb7, 0x1d, 0xc8, 0x9c, 0x49, 0xe3, 0x36,
    0x19, 0xcc, 0x66, 0xb3, 0xe7, 0x32, 0x98, 0x4d, 0x30, 0xe5, 0x4f, 0x9a, 0xce, 0x1b, 0xb1, 0x64,
    0x72, 0xa7, 0x0d, 0xd8, 0x8c, 0x59, 0xf3, 0x26, 0x5b, 0x8e, 0x24, 0xf1, 0xa5, 0x70, 0xda, 0x0f,
    0x20, 0xf5, 0x5f, 0x8a, 0xde, 0x0b, 0xa1, 0x74, 0x09, 0xdc, 0x76, 0xa3, 0xf7, 0x22, 0x88, 0x5d,
    0xd6, 0x03, 0xa9, 0x7c, 0x28, 0xfd, 0x57, 0x82, 0xff, 0x2a, 0x80, 0x55, 0x01, 0xd4, 0x7e, 0xab,
    0x84, 0x51, 0xfb, 0x2e, 0x7a, 0xaf, 0x05, 0xd0, 0xad, 0x78, 0xd2, 0x07, 0x53, 0x86, 0x2c, 0xf9,
]

def _crc8_dvb_s2(crc, ch):
    return crc8_table[crc^ch]