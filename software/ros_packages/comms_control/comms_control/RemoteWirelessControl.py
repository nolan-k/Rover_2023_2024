import typing
import subprocess
from enum import Enum

class InterfaceStatus:
    signalLevel: typing.Optional[int] #Recieved signal level dBm
    noiseLevel: typing.Optional[int] #noise level dBm
    frequency: typing.Optional[int] #channel freq in MHz
    channel: typing.Optional[int] #channel #
    rxmcs: typing.Optional[str] #download modulation coding scheme
    txmcs: typing.Optional[str] #upload modulation coding scheme
    rxbitrate: typing.Optional[float] #download bitrate Mbps
    txbitrate: typing.Optional[float] #upload bitrate Mbps
    txpower: typing.Optional[float] #transmit power dBm
    channelWidth: typing.Optional[int] #channel width MHz

    def __init__(
                self,
                signalLevel: typing.Optional[int] = None,
                noiseLevel: typing.Optional[int] = None,
                frequency: typing.Optional[int] = None,
                channel: typing.Optional[int] = None,
                rxmcs: typing.Optional[str] = None,
                txmcs: typing.Optional[str] = None,
                rxbitrate: typing.Optional[float] = None,
                txbitrate: typing.Optional[float] = None,
                txpower: typing.Optional[float] = None,
                channelWidth: typing.Optional[int] = None
                 ):
        self.signalLevel = signalLevel
        self.noiseLevel = noiseLevel
        self.frequency = frequency
        self.channel = channel
        self.channelWidth = channelWidth
        self.rxmcs = rxmcs
        self.txmcs = txmcs
        self.rxbitrate = rxbitrate
        self.txbitrate = txbitrate
        self.txpower = txpower
        self.channelWidth = channelWidth

    def __str__(self) -> str:
        return \
        f"Signal Level (dBm): {self.signalLevel}\n" + \
        f"Noise Level (dBm): {self.noiseLevel}\n" + \
        f"Frequency MHz (dBm) {self.frequency}\n" + \
        f"Channel: {self.channel}\n" + \
        f"Channel Width (MHz): {self.channelWidth}\n" + \
        f"RX MCS: {self.rxmcs}\n" + \
        f"TX MCS: {self.txmcs}\n" + \
        f"RX Bitrate: {self.rxbitrate}\n" + \
        f"TX Bitrate: {self.txbitrate}\n" + \
        f"TX Power (dBm): {self.txpower}"



class WirelessInterface:

    class InterfaceType(Enum):
        GENERIC_IW = 0
        MM_HALOW = 1
        UBIQUITI_AIROS = 2

    remoteAddr: str
    password: str
    username: str
    interfaceName: str
    type: InterfaceType

    def __init__(self, remoteAddr: str = "", username: str = "", interfaceName: str = "", password: str = "", type: InterfaceType = InterfaceType.GENERIC_IW):
        self.remoteAddr = remoteAddr
        self.interfaceName = interfaceName
        self.username = username
        self.password = password
        self.type = type  

    #Gets status information about this interface.
    def getStatus(self) -> InterfaceStatus:

        def sshRun(command: str) -> subprocess.CompletedProcess[str]:
            return subprocess.run(f'ssh {self.username}@{self.remoteAddr} "{command}"', text=True, capture_output=True)

        def intOrNone(input: str) -> typing.Optional[int]:
            try:
                return int(input)
            except ValueError:
                return None
            
        def floatOrNone(input: str) -> typing.Optional[float]:
            try:
                return float(input)
            except ValueError:
                return None

        result = InterfaceStatus()
        
        if self.type == WirelessInterface.InterfaceType.GENERIC_IW:
            #use iw to get interface info. This should contain the channel, frequency, channel width, and tx power.
            output = sshRun(f"iw dev {self.interfaceName} info")

            if output.returncode == 0:
                tokens = output.stdout.split()
                for i in range(0, len(tokens)):
                    if (tokens[i] == 'channel'):
                        i += 1
                        result.channel = intOrNone(tokens[i])
                        i += 1
                        result.frequency = intOrNone(tokens[i].removeprefix('('))
                    elif (tokens[i] == 'width:'):
                        i += 1
                        result.channelWidth = intOrNone(tokens[i])
                    elif (tokens[i] == 'txpower'):
                        i += 1
                        result.txpower = floatOrNone(tokens[i])
            else:
                print(f"Failed to connect to {self.username}@{self.remoteAddr} over ssh.")

            #use iw to get link info. Should contain signal level, bitrates, and mcs info
            output = sshRun(f"iw dev {self.interfaceName} link")

            if output.returncode == 0:
                lines = output.stdout.splitlines()
                for l in lines:
                    tokens = l.split()
                    if (tokens[0] == 'signal:'):
                        result.channel = intOrNone(tokens[1])
                    elif (tokens[0] == 'rx bitrate:'):
                        result.rxbitrate = floatOrNone(tokens[1])
                        result.rxmcs = str.join(' ', tokens[1:len(tokens)])
                    elif (tokens[0] == 'tx bitrate:'):
                        result.txbitrate = floatOrNone(tokens[1])
                        result.txmcs = str.join(' ', tokens[1:len(tokens)])
            else:
                print(f"Failed to connect to {self.username}@{self.remoteAddr} over ssh.")

            #attempt to get noise level from /proc/net/wireless
            output = sshRun(f"cat /proc/net/wireless")

            if output.returncode == 0:
                lines = output.stdout.splitlines()
                for l in lines:
                    tokens = l.split()
                    if (tokens[0] == self.interfaceName+':'):
                        result.noiseLevel = intOrNone(tokens[4])
            else:
                print(f"Failed to connect to {self.username}@{self.remoteAddr} over ssh.")

        elif self.type == WirelessInterface.InterfaceType.MM_HALOW:

            output = sshRun(f"morse_cli -i {self.interfaceName} stats")

            lines = output.stdout.splitlines()

            for l in lines:
                tokens = l.split(':')
                if tokens[0] == "Received Power (dBm)":
                    result.signalLevel = intOrNone(tokens[1])
                elif tokens[0] == "Noise dBm":
                    result.noiseLevel = intOrNone(tokens[1])
                elif tokens[0] == "Current RF frequency Hz":
                    result.frequency = intOrNone(tokens[1])
                    if type(result.frequency) == int:
                        result.frequency //= 1000000
                elif tokens[0] == "Current Operating BW MHz":
                    result.channelWidth = intOrNone(tokens[1])

            output = sshRun(f"iwinfo {self.interfaceName} info") 

            for l in lines:
                tokens = l.split(':')
                if tokens[0].strip() == "Tx-Power":
                    result.txpower = intOrNone(tokens[1])
                elif tokens[0].strip() == "Mode":
                    result.channel = intOrNone(tokens[3])
                elif tokens[0].strip() == "Bit Rate":
                    result.rxbitrate = floatOrNone(tokens[1])  

        elif self.type == WirelessInterface.InterfaceType.UBIQUITI_AIROS:
            #TODO parse airos status
                     

        return result


    