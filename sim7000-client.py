from os import system
import serial
import subprocess
from time import sleep,perf_counter
import distutils.util
from datetime import datetime
import json
from timer import Timer

import pubnub
from pubnub.pnconfiguration import PNConfiguration
from pubnub.pubnub import PubNub
from pubnub.callbacks import SubscribeCallback
from pubnub.enums import PNOperationType, PNStatusCategory

pnconfig = PNConfiguration()
pnconfig.subscribe_key = "sub-c-cf845704-8def-11ea-8e98-72774568d584"
pnconfig.publish_key = "pub-c-8f52ff44-41bb-422c-a0c0-a63167077c6d"
pnconfig.uuid = "Client-scooter1"
pnconfig.ssl = False
pubnub = PubNub(pnconfig)
CHANNEL_ID = "Robotronix"

BAUDRATE = 115200
SECONDS_BETWEEN_READS = 1
STREAM_DELAY = 5
INIT_DELAY = 1
index = 0
stream_index = 0
DATA_POINT = 2 # GPS lat/lgt recorded before transmission
scooter1_activated = False
scooter1_moved = False
scooterAlarm = False
serverActivation = False
userActivation = False

class MySubscribeCallback(SubscribeCallback):
    def status(self, pubnub, status):
        # The status object returned is always related to subscribe but could contain
        # information about subscribe, heartbeat, or errors
        # use the operationType to switch on different options
        if status.operation == PNOperationType.PNSubscribeOperation \
                or status.operation == PNOperationType.PNUnsubscribeOperation:
            if status.category == PNStatusCategory.PNConnectedCategory:
                pass
                # This is expected for a subscribe, this means there is no error or issue whatsoever
            elif status.category == PNStatusCategory.PNReconnectedCategory:
                pass
                # This usually occurs if subscribe temporarily fails but reconnects. This means
                # there was an error but there is no longer any issue
            elif status.category == PNStatusCategory.PNDisconnectedCategory:
                pass
                # This is the expected category for an unsubscribe. This means there
                # was no error in unsubscribing from everything
            elif status.category == PNStatusCategory.PNUnexpectedDisconnectCategory:
                pass
                # This is usually an issue with the internet connection, this is an error, handle
                # appropriately retry will be called automatically
            elif status.category == PNStatusCategory.PNAccessDeniedCategory:
                pass
                # This means that PAM does not allow this client to subscribe to this
                # channel and channel group configuration. This is another explicit error
            else:
                pass
                # This is usually an issue with the internet connection, this is an error, handle appropriately
                # retry will be called automatically
        elif status.operation == PNOperationType.PNSubscribeOperation:
            # Heartbeat operations can in fact have errors, so it is important to check first for an error.
            # For more information on how to configure heartbeat notifications through the status
            # PNObjectEventListener callback, consult http://www.pubnub.com/docs/python/api-reference-configuration#configuration
            if status.is_error():
                pass
                # There was an error with the heartbeat operation, handle here
            else:
                pass
                # Heartbeat operation was successful
        else:
            pass
            # Encountered unknown status type

    def presence(self, pubnub, presence):
        pass  # handle incoming presence data

    def message(self, pubnub, message):
        global serverActivation
        global userActivation
        receivedMessage = json.dumps(message.message)
        if "s_act" and "u_act" in receivedMessage:
            print(receivedMessage)
            if "s1" in receivedMessage:
                serverActivation = True
            else:
                serverActivation = False
            print("server:{}".format(serverActivation))

            if "u1" in receivedMessage:
                userActivation = True
            else:
                userActivation = False
            print("user:{}".format(userActivation))
        pass  # handle incoming messages

    def signal(self, pubnub, signal):
        pass # handle incoming signals

pubnub.add_listener(MySubscribeCallback())

def str2bool_util(inp):
    str2int = distutils.util.strtobool(inp)
    boolean = bool(str2int)
    return boolean

def publish_callback(result, status):
    pass
    # Handle PNPublishResult and PNStatus
# Start PPPD
def openPPPD():
    # Check if PPPD is already running by looking at syslog output
    print("Opening PPPD...")
    output1 = subprocess.check_output("cat /var/log/syslog | grep pppd | tail -1", shell=True)
    if "secondary DNS address" not in output1 and "locked" not in output1:
        while True:
            # Start the "fona" process
            print("starting fona process...")
            subprocess.call("sudo pon fona", shell=True)
            sleep(2)
            output2 = subprocess.check_output("cat /var/log/syslog | grep pppd | tail -1", shell=True)
#             print(output2)
            if "script failed" not in output2:
                break
#     # Make sure the connection is working
    while True:
        print("Connection check...")
        output2 = subprocess.check_output("cat /var/log/syslog | grep pppd | tail -1", shell=True)
#         output3 = subprocess.check_output("cat /var/log/syslog | grep pppd | tail -3", shell=True)
#         print("Out2:{}".format(output2))
#         print("Out3:{}".format(output3))
#         if "secondary DNS address" in output2 or "DNS address" in output3:
        if "secondary DNS address" in output2:
            print("Connection is ready...Device is online...")
            return True

# Stop PPPD
def closePPPD():
    print ("\nTurning off cell connection using sudo poff fona...")
    # Stop the "fona" process
    subprocess.call("sudo poff fona", shell=True)
    # Make sure connection was actually terminated
    while True:
        output = subprocess.check_output("cat /var/log/syslog | grep pppd | tail -1", shell=True)
        if "Exit" in output:
            print("pppd is now close...")
            return True

# Check for a GPS fix
def checkForFix():
    # Start the serial connection SIM7000E
    ser=serial.Serial('/dev/ttyS0', BAUDRATE, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=1)
    # Turn on the GPS
    ser.write(b"AT+CGNSPWR=1\r")
    ser.write(b"AT+CGNSPWR?\r")
    while True:
        response = ser.readline()
        if b"1" in response: # remove the whitespace before 1 for SIM7000E
            break
    # Ask for the navigation info parsed from NMEA sentences
    ser.write(b"AT+CGNSINF\r")
    while True:
            response = ser.readline()
            # Check if a fix was found
            if b"+CGNSINF: 1,1," in response:
                return True
            # If a fix wasn't found, wait and try again
            if b"+CGNSINF: 1,0," in response:
                sleep(5)
                ser.write(b"AT+CGNSINF\r")
                print ("Unable to find fix. still looking for fix...")
            else:
                ser.write(b"AT+CGNSINF\r")

# Read the GPS data for Latitude and Longitude
def getCoord():
    # Start the serial connection SIM7000E
    ser=serial.Serial('/dev/ttyS0', BAUDRATE, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=1)

    ser.write(b"AT+CGNSINF\r")
    while True:
        response = ser.readline()
        if b"+CGNSINF: 1," in response:
            # Split the reading by commas and return the parts referencing lat and long
            array = response.split(b",")
            lat = array[3]
            # print lat
            lon = array[4]
            # print lon
            return (lat,lon) # return lat & lon value in byte form

# Read the GNSS Navigation Information by parsing the complete NMEA sentence
def getNavigationInfo():
    # Start the serial connection SIM7000E
    ser=serial.Serial('/dev/ttyS0', BAUDRATE, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=1)
    ser.write(b"AT+CGNSINF\r")
    datetime_objc = datetime.now()
    print(datetime_objc)

    while True:
        response = ser.readline()
        if b"+CGNSINF: 1," in response:
            # Split the reading by commas
            array = response.split(b",")
            grun = array[0] # GNSS run status
            sfix = array[1] # Fix status
            utct = array[2] # UTC date & time
            clat = array[3] # latitude
            clon = array[4] # longitude
            altd = array[5] # MSL altitude
            spdg = array[6] # speed over ground
            csog = array[7] # course over ground
            mfix = array[8] # fix mode
            rsv1 = array[9] # reserved1
            hdop = array[10] # HDOP horizontal dilution of precision
            pdop = array[11] # PDOP position (3D) dilution of precision
            vdop = array[12] # VDOP vertical dilution of precision
            rsv2 = array[13] # reserved2
            gnsv = array[14] # GNSS Satellites in View
            gnsu = array[15] # GNSS Satellites in Use
            glns = array[16] # GLONASS Satellites Used
            rsv3 = array[17] # reserved3
            cnom = array[18] # C/N0 max
            hpa0 = array[19] # Horizontal Position Accuracy
            vpa0 = array[20] # Vertical Position Accuracy

            # print("MSL altitude:{}m = {}ft".format(altd,round(float(altd)/0.3048),4))
            print("Speed over Ground:{} km/h".format(spdg))
            # print("Course over Ground:{} degrees".format(csog))
            # print("HDOP:{}".format(hdop))
            # print("PDOP:{}".format(pdop))
            # print("VDOP:{}".format(vdop))
            print("GNSS Satellites in View:{}".format(gnsv))
            print("GNSS Satellites in Use:{}".format(gnsu))
            print("GLONASS in Use:{}".format(glns))
            # print("C/N0 max:{} dBHz".format(cnom))
            # print("HPA:{} m".format(hpa0))
            return spdg

def main_with_pppd():
    global stream_index
    # Start the program by opening the cellular connection and creating a bucket for our data
    if openPPPD():
        # Wait long enough for the request to complete
        for c in range(INIT_DELAY):
            print ("Starting in T-minus {} second".format(INIT_DELAY-c))
            sleep(1)
        while True:
            # Close the cellular connection
            if closePPPD():
                index=0 # reset counter after closing connection
                sleep(1)
            # The range is how many data points we'll collect before streaming
            for i in range(DATA_POINT):
                # Make sure there's a GPS fix
                if checkForFix():
                    # Get lat and long
                    print("i = {}".format(i))
                    if getCoord():
                        index+=1
                        latitude, longitude = getCoord()
                        coord = "lat:" + str(latitude) + "," + "lgt:" + str(longitude)
                        print (coord) # this is string formatted lat/lon
                        print("Saving read #{} into buffer.".format(index))
                        sleep(SECONDS_BETWEEN_READS) # 1 second
                    # Turn the cellular connection on every 2 reads
                    if i == DATA_POINT-1:
                        sleep(1)
                        print ("opening connection")
                        if openPPPD():
                            stream_index+=1


def main_without_pppd():
    global index
    global serverActivation
    global userActivation
    s1_last_seen = ""

    idle_time = 0.0
    active_time = 0.0
    idle_flag = True
    active_flag = True

    for c in range(INIT_DELAY):
        print ("Starting in T-minus {} second".format(INIT_DELAY-c))
        sleep(1)

    while True:
        # start listening to the channel from incoming messages
        pubnub.subscribe().channels(CHANNEL_ID).execute()
        print("\n\nStream index:{} \n".format(index))

        # do the logic for anti-theft system here
        #  /__\  ( \( )(_  _)(_  _)    (_  _)( )_( )( ___)( ___)(_  _)
        # /(  )\  )  (   )(   _)(_       )(   ) _ (  )__)  )__)   )(
        #(__)(__)(_)\_) (__) (____)     (__) (_) (_)(____)(_)    (__)

        scooter1_activated = (serverActivation and userActivation)
        if scooter1_activated:
            if not idle_flag:
                idle_flag = True
                idle_end = perf_counter()
                idle_time = idle_end - idle_start
                print("//////////////////////////Was Idling for {} sec".format(idle_time))
                active_flag = True

            if active_flag:
                active_start = perf_counter()
                active_flag = False
            print("Scooter is now active for {:.1f} sec".format(perf_counter()-active_start))

        else:
            if idle_flag:
                idle_start = perf_counter()
                idle_flag = False
            print("Scooter is now idle for {:.1f} sec".format(perf_counter()-idle_start))

            if not active_flag:
                active_end = perf_counter()
                active_time = active_end - active_start
                print("////////////////////////// Was Active for {} sec".format(active_time))
                active_flag = True


        print("ACTIVATION_S1:{}".format(scooter1_activated))
        groundSpeed = getNavigationInfo()
        gndSpeed = float(groundSpeed)

        if gndSpeed is not 0.00:
            scooter1_moved = True
        else:
            scooter1_moved = False

        if not scooter1_activated and scooter1_moved:
            scooterAlarm = True
        else:
            scooterAlarm = False

        print("standby_alarm_S1:{}".format(scooterAlarm))

        # Make sure there's a GPS fix
        if checkForFix():
            # Get lat and long
            if getCoord():
                index+=1
                latitude, longitude = getCoord() # live coordinates
                # in_activated = input("active-s1?:")
                # scooter1_activated=str2bool_util(in_activated)
                if not scooter1_activated:
                    s1_last_seen = float(longitude),float(latitude)
                else:
                    s1_last_seen = ""

                coord = "lat:" + str(latitude) + "," + "lng:" + str(longitude)
                print (coord)
                # create JSON dictionary
                dictionary =    {
                                "s1_index":         float(index),
                                "s1_speed":         float(gndSpeed+30),
                                "s1_latitude":      float(latitude),
                                "s1_longitude":     float(longitude),
                                "s1_activated":     bool(scooter1_activated),
                                "s1_moved":         bool(scooter1_moved),
                                "s1_last_known":    s1_last_seen
                                }
                pubnub.publish().channel(CHANNEL_ID).message(dictionary).pn_async(publish_callback)

                print("\nNext stream in:\n")
                for c in range(STREAM_DELAY):
                    print(STREAM_DELAY-c, end = ' ')
                    # print ("{} second".format(STREAM_DELAY-c))
                    sleep(SECONDS_BETWEEN_READS)


if __name__ == "__main__":
    try:
        ser=serial.Serial('/dev/ttyS0', BAUDRATE, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=1)
        main_without_pppd()
    except KeyboardInterrupt:
        print("Turning off GPS\n")
