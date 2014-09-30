# timestanp the audio buffer
Header header
# current frequency of the audio interface
uint16 frequency
# channel order properties :
uint8 CHANNEL_FRONT_LEFT=0
uint8 CHANNEL_FRONT_CENTER=1
uint8 CHANNEL_FRONT_RIGHT=2
uint8 CHANNEL_REAR_LEFT=3
uint8 CHANNEL_REAR_CENTER=4
uint8 CHANNEL_REAR_RIGHT=5
uint8 CHANNEL_SURROUND_LEFT=6
uint8 CHANNEL_SURROUND_RIGHT=7
uint8 CHANNEL_SUBWOOFER=8
uint8 CHANNEL_LFE=9
# channel order of the current buffer
uint8[] channelMap
# interlaced data of the audio buffer
int16[] data