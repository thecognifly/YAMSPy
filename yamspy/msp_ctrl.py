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

def receive_raw_msg(local_read, logging, size, timeout = 0.1):
    """Receive multiple bytes at once when it's not a jumbo frame.

    Returns
    -------
    bytes
        data received
    """
    timeout = time.time() + timeout
    msg_header = ''
    while True:
        if time.time() >= timeout:
            logging.warning("Timeout occured when receiving a message")
            return msg_header
        msg_header = local_read()
        if msg_header:
            if ord(msg_header) == 36: # $
                break
    msg = local_read(size - 1) # -1 to compensate for the $

    return msg_header + msg

def receive_msg(local_read, logging):
    """Receive an MSP message from the serial port
    Based on betaflight-configurator (https://git.io/fjRAz)

    Returns
    -------
    dict
        dataHandler with the received data pre-parsed
    """

    dataHandler = dataHandler_init.copy()
    received_bytes = receive_raw_msg(local_read, logging, size=3)
    if not received_bytes:
        logging.warning("receive_msg got nothing...")
        return dataHandler
    dataHandler['last_received_timestamp'] = time.time()

    di = 0
    while True:
        try:
            data = received_bytes[di]
            di += 1
            logging.debug("State: {1} - byte received (at {0}): {2}".format(dataHandler['last_received_timestamp'], 
                                                                    dataHandler['state'], 
                                                                    data))
        except IndexError:
            # Instead of crashing everything, let's just ignore this msg...
            # ... and hope for the best :)
            logging.debug('IndexError detected on state: {}'.format(dataHandler['state']))
            dataHandler['state'] = -1
            break # Sends it to the error state

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
                break # sends it to the error state

        elif dataHandler['state'] == 2: # direction (should be >)
            dataHandler['unsupported'] = 0
            if (data == 33): # !
                # FC reports unsupported message error
                logging.debug('FC reports unsupported message error.')
                dataHandler['unsupported'] = 1
                break # sends it to the error state
            else:
                if (data == 62): # > FC to PC
                    dataHandler['message_direction'] = 1
                elif (data == 60): # < PC to FC
                    dataHandler['message_direction'] = 0
                    
                if dataHandler['msp_version'] == 1:
                    dataHandler['state'] = 3
                    received_bytes += local_read(2)
                elif dataHandler['msp_version'] == 2:
                    dataHandler['state'] = 2.1
                    received_bytes += local_read(5)

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
                received_bytes += local_read(dataHandler['message_length_expected']+2) # +2 for CRC
            else:
                dataHandler['state'] = 9
                received_bytes += local_read(2) # 2 for CRC

        elif dataHandler['state'] == 4:
            dataHandler['code'] = data
            dataHandler['message_checksum'] ^= data

            if dataHandler['message_length_expected'] > 0:
                # process payload
                if dataHandler['messageIsJumboFrame']:
                    dataHandler['state'] = 5
                    received_bytes += local_read()
                else:
                    dataHandler['state'] = 7
                    received_bytes += local_read(dataHandler['message_length_expected']+1) # +1 for CRC
            else:
                # no payload
                dataHandler['state'] = 9
                received_bytes += local_read()

        elif dataHandler['state'] == 5:
            # this is a JumboFrame
            dataHandler['message_length_expected'] = data

            dataHandler['message_checksum'] ^= data

            dataHandler['state'] = 6
            received_bytes += local_read()

        elif dataHandler['state'] == 6:
            # calculates the JumboFrame size
            dataHandler['message_length_expected'] +=  256 * data
            logging.debug("JumboFrame message_length_expected: {}".format(dataHandler['message_length_expected']))
            # There's no way to check for transmission errors here...
            # In the worst scenario, it will try to read 255 + 256*255 = 65535 bytes

            dataHandler['message_checksum'] ^= data

            dataHandler['state'] = 7
            received_bytes += local_read(dataHandler['message_length_expected']+1) # +1 for CRC

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
                        return dataHandler
                    else:
                        # wrong checksum
                        logging.debug('Code: {0} - crc failed (received {1}, calculated {2})'.format(dataHandler['code'], 
                                                                                                    data,
                                                                                                    dataHandler['message_checksum']))
                        dataHandler['crcError'] = True
                        break # sends it to the error state
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
                        return dataHandler
                    else:
                        # wrong checksum
                        logging.debug('Code: {0} - crc failed (received {1}, calculated {2})'.format(dataHandler['code'], 
                                                                                                    data,
                                                                                                    dataHandler['message_checksum']))
                        dataHandler['crcError'] = True
                        break # sends it to the error state

    # it means an error occurred
    logging.debug('Error detected on state: {}'.format(dataHandler['state']))
    dataHandler['packet_error'] = 1

    return dataHandler


def prepare_RAW_msg(code, data=[]):
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
    if code <= 255: # MSP V1
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

    elif code > 255: # MSP V2
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