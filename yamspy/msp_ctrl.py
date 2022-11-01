import time

JUMBO_FRAME_SIZE_LIMIT = 255


dataHandler_init = {
    'msp_version':                1,
    'state':                      0,
    'message_direction':          -1,
    'code':                       0,
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

    'last_received_timestamp':   None
}


read_buffer = b''
def _read(local_read):
    def read(size=None, buffer=None):
        global read_buffer
        if buffer:
            read_buffer = buffer
            return
            
        output = b''
        if size:
            while True:
                output += read_buffer[:size]
                read_buffer = read_buffer[size:]
                size -= len(output)
                if size > 0:
                    read_buffer += local_read() # read (try) everything in the seria/socket buffer
                else:
                    break
        else:
            if len(read_buffer)==0:
                read_buffer += local_read() # read (try) everything in the seria/socket buffer
            output += read_buffer
            read_buffer = b''
        return output
    return read

def receive_raw_msg(local_read, logging, timeout_exception, size, timeout = 10):
    """Receive multiple bytes at once when it's not a jumbo frame.
    Returns
    -------
    bytes
        data received
    """
    local_read = _read(local_read)
    msg_header = b''
    timeout = time.time() + timeout
    while True:
        if time.time() >= timeout:
            logging.warning("Timeout occured when receiving a message")
            raise timeout_exception("receive_raw_msg timeout")
        msg_header = local_read(size=1)
        if msg_header:
            if ord(msg_header) == 36: # $
                break
    msg = local_read(size=(size - 1)) # -1 to compensate for the $
    return msg_header + msg

def receive_msg(local_read, logging, output_raw_bytes=False):
    """Receive an MSP message from the serial port
    Based on betaflight-configurator (https://git.io/fjRAz)

    Returns
    -------
    dict
        dataHandler with the received data pre-parsed
    """
    local_read = _read(local_read)
    dataHandler = dataHandler_init.copy()
    if output_raw_bytes:
        raw_bytes = b''

    di = 0
    while True:
        try:
            if di == 0:
                received_bytes = memoryview(local_read()) # it will read everything from the buffer
                if received_bytes:
                    dataHandler['last_received_timestamp'] = time.time()
                    data = received_bytes[di]
                    if output_raw_bytes:
                        raw_bytes += received_bytes[di:di+1]
                else:
                    dataHandler['packet_error'] = 1
                    break
            else:
                data = received_bytes[di]
                if output_raw_bytes:
                    raw_bytes += received_bytes[di:di+1]

            di += 1
            logging.debug(f"State: {dataHandler['state']} - byte received (at {dataHandler['last_received_timestamp']}): {data}")
        except IndexError:
            logging.debug('IndexError detected on state: {}'.format(dataHandler['state']))
            di = 0 # reads more data
            continue

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
                logging.debug('Something went wrong, no M received.')
                dataHandler['packet_error'] = 1
                break # sends it to the error state

        elif dataHandler['state'] == 2: # direction (should be >)
            dataHandler['unsupported'] = 0
            if (data == 33): # !
                # FC reports unsupported message error
                logging.debug('FC reports unsupported message error.')
                dataHandler['unsupported'] = 1
                dataHandler['packet_error'] = 1
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
        local_read(buffer=bytes(received_bytes[di:])) # regurgitates unread stuff :)

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

        bufView = bytearray([0]*size)

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
        bufView = bytearray([0]*size)
        bufView[0] = 36 #$ 
        bufView[1] = 88 #X
        bufView[2] = 60 #<
        bufView[3] = 0 #flag: reserved, set to 0
        bufView[4] = code & 0xFF #code lower byte
        bufView[5] = (code & 0xFF00) >> 8 #code upper byte
        bufView[6] = len_data & 0xFF #len_data lower byte
        bufView[7] = (len_data & 0xFF00) >> 8 #len_data upper byte
        for di in range(len_data):
            bufView[8+di] = data[di]
        for si in range(3, size-1):
            checksum = _crc8_dvb_s2(checksum, bufView[si])
        bufView[-1] = checksum

    else:
        return []

    return bufView


def _crc8_dvb_s2(crc, ch):
    """CRC for MSPV2
    *copied from inav-configurator
    """
    crc ^= ch
    for _ in range(8):
        if (crc & 0x80):
            crc = ((crc << 1) & 0xFF) ^ 0xD5
        else:
            crc = (crc << 1) & 0xFF
    return crc