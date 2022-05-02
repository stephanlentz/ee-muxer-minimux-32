#!/bin/bash
#Set command interface parameter slot 16 page 0 bit 31
#./bin/v6test -c -a 0x17 -d 0x1F0010


# Set slotmask to first command line parameter
./bin/v6test -S $1
# Second command line parameter gives a space separated list of A6 slots
module=0
for slot in $2
do
echo "Init a6 $module in slot $slot"

echo "Set mute after write time"
./bin/v6test -s $slot -o 6 -p 15 -i 2 -d 0x00020

echo "Ignore additional data at RS485 (driver problem with short cables)"
./bin/v6test -s $slot -o 6 -p 14 -i 36 -d 1

#-------------------------------------------------------------
# Way based filter 
#-------------------------------------------------------------

# Bit position on para in stream, mirrored on all pages
#./bin/v6test -s $slot -o 6 -p 31 -i 2 -d 8

# Muxch 0,1 way triggered
#./bin/v6test -s $slot -o 6 -p 31 -i 3 -d 0x03

# NCO factor min
#./bin/v6test -s 16 -o 2 -p 129 -d 1

#--- Write Table ---------------------------------------------

export SLOT=$slot
echo "Programming mux-table:"
echo "...clear muxing_allowed"
./bin/v6test -s $slot -o 8 -p 13 -i 0 -d 0
echo "...clear config valid"
./serial/write8fast 0x1D 0
echo "...write table lines"
# Fremdzunge:
# Geometry:
#
#         0     2     4    6  ..   28   30
#       
#       
#       
#       1    3     5     7   ..  29  31     
#
# Since the rows are at wide distance the best try is
# Send at 0, rcv 1/2
# Send at 2, rcv 3/4
#...
# Send at 28, rcv 29/30
# and in addition:
# Send at 30, rcv 31/28
# In total 16 muxch
#
#                       idx en_snd[32] rcv_n/p[8/8]
#                                                  snd rcvn rcvp
#./serial/writetableline  0  0x00000001  0x0102  #   0   1   2
#./serial/writetableline  1  0x00000004  0x0304  #   2   3   4
#./serial/writetableline  2  0x00000010  0x0506  #   4   5   6
#./serial/writetableline  3  0x00000040  0x0708  #   6   7   8
#./serial/writetableline  4  0x00000100  0x090A  #   8   9   10
#./serial/writetableline  5  0x00000400  0x0B0C  #   10  11  12
#./serial/writetableline  6  0x00001000  0x0D0E  #   12  13  14
#./serial/writetableline  7  0x00004000  0x0F10  #   14  15  16
#./serial/writetableline  8  0x00010000  0x1112  #   16  17  18
#./serial/writetableline  9  0x00040000  0x1314  #   18  19  20
#./serial/writetableline 10  0x00100000  0x1516  #   20  21  22
#./serial/writetableline 11  0x00400000  0x1718  #   22  23  24
#./serial/writetableline 12  0x01000000  0x191A  #   24  25  26
#./serial/writetableline 13  0x04000000  0x1B1C  #   26  27  28
#./serial/writetableline 14  0x10000000  0x1D1E  #   28  29  30
#./serial/writetableline 15  0x40000000  0x1F1C  #   30  31  28

# Eigene Zunge:
# Geometry:
#
#         0   2   4   6  ..   28  30
#           1   3   5   7   ..  29  31     
#
# Send at 0, rcv 1/2
# Send at 1, rcv 2/3
#...
# Send at 28, rcv 29/30
# Send at 29, rcv 30/31
# In total 30 muxch
#
#                       idx en_snd[32] rcv_n/p[8/8]
#                                                  snd  rcvn rcvp
./serial/writetableline  0  0x00000001  0x0102  #   0     1    2 
./serial/writetableline  1  0x00000002  0x0203  #   1     2    3
./serial/writetableline  2  0x00000004  0x0304  #   2     3    4
./serial/writetableline  3  0x00000008  0x0405  #   3     4    5
./serial/writetableline  4  0x00000010  0x0506  #   4     5    6
./serial/writetableline  5  0x00000020  0x0607  #   5     6    7
./serial/writetableline  6  0x00000040  0x0708  #   6     7    8
./serial/writetableline  7  0x00000080  0x0809  #   7     8    9
./serial/writetableline  8  0x00000100  0x090A  #   8     9   10
./serial/writetableline  9  0x00000200  0x0A0B  #   9    10   11
./serial/writetableline 10  0x00000400  0x0B0C  #  10    11   12
./serial/writetableline 11  0x00000800  0x0C0D  #  11    12   13
./serial/writetableline 12  0x00001000  0x0D0E  #  12    13   14
./serial/writetableline 13  0x00002000  0x0E0F  #  13    14   15
./serial/writetableline 14  0x00004000  0x0F10  #  14    15   16
./serial/writetableline 15  0x00008000  0x1011  #  15    16   17
./serial/writetableline 16  0x00010000  0x1112  #  16    17   18
./serial/writetableline 17  0x00020000  0x1213  #  17    18   19
./serial/writetableline 18  0x00040000  0x1314  #  18    19   20
./serial/writetableline 19  0x00080000  0x1415  #  19    20   21
./serial/writetableline 20  0x00100000  0x1516  #  20    21   22
./serial/writetableline 21  0x00200000  0x1617  #  21    22   23
./serial/writetableline 22  0x00400000  0x1718  #  22    23   24
./serial/writetableline 23  0x00800000  0x1819  #  23    24   25
./serial/writetableline 24  0x01000000  0x191A  #  24    25   26
./serial/writetableline 25  0x02000000  0x1A1B  #  25    26   27
./serial/writetableline 26  0x04000000  0x1B1C  #  26    27   28
./serial/writetableline 27  0x08000000  0x1C1D  #  27    28   29
./serial/writetableline 28  0x10000000  0x1D1E  #  28    29   30
./serial/writetableline 29  0x20000000  0x1E1F  #  29    30   31
#oder:
# Send at 0, rcv 1/2
# Send at 1, rcv 3/2
# Send at 2, rcv 3/4
# Send at 3, rcv 5/4
#...
# Send at 28, rcv 29/30
# Send at 29, rcv 31/30
# In total 30 muxch
#
#                       idx en_snd[32] rcv_n/p[8/8]
#                                                  snd  rcvn rcvp
#./serial/writetableline  0  0x00000001  0x0102  #   0     1    2 
#./serial/writetableline  1  0x00000002  0x0302  #   1     3    2
#./serial/writetableline  2  0x00000004  0x0304  #   2     3    4
#./serial/writetableline  3  0x00000008  0x0504  #   3     5    4
#./serial/writetableline  4  0x00000010  0x0506  #   4     5    6
#./serial/writetableline  5  0x00000020  0x0706  #   5     7    6
#./serial/writetableline  6  0x00000040  0x0708  #   6     7    8
#./serial/writetableline  7  0x00000080  0x0908  #   7     9    8
#./serial/writetableline  8  0x00000100  0x090A  #   8     9   10
#./serial/writetableline  9  0x00000200  0x0B0A  #   9    11   10
#./serial/writetableline 10  0x00000400  0x0B0C  #  10    11   12
#./serial/writetableline 11  0x00000800  0x0D0C  #  11    13   12
#./serial/writetableline 12  0x00001000  0x0D0E  #  12    13   14
#./serial/writetableline 13  0x00002000  0x0F0E  #  13    15   14
#./serial/writetableline 14  0x00004000  0x0F10  #  14    15   16
#./serial/writetableline 15  0x00008000  0x1110  #  15    17   16
#./serial/writetableline 16  0x00010000  0x1112  #  16    17   18
#./serial/writetableline 17  0x00020000  0x1312  #  17    19   18
#./serial/writetableline 18  0x00040000  0x1314  #  18    19   20
#./serial/writetableline 19  0x00080000  0x1514  #  19    21   20
#./serial/writetableline 20  0x00100000  0x1516  #  20    21   22
#./serial/writetableline 21  0x00200000  0x1716  #  21    23   22
#./serial/writetableline 22  0x00400000  0x1718  #  22    23   24
#./serial/writetableline 23  0x00800000  0x1918  #  23    25   24
#./serial/writetableline 24  0x01000000  0x191A  #  24    25   26
#./serial/writetableline 25  0x02000000  0x1B1A  #  25    27   26
#./serial/writetableline 26  0x04000000  0x1B1C  #  26    27   28
#./serial/writetableline 27  0x08000000  0x1D1C  #  27    29   28
#./serial/writetableline 28  0x10000000  0x1D1E  #  28    29   30
#./serial/writetableline 29  0x20000000  0x1E1E  #  29    31   30


echo "Save persistent data (if supported)"
./serial/save_pers
echo "Set end_mux to 29"
./bin/v6test -s $slot -o 6 -p 18 -i 0 -d 29
./bin/v6test -s $slot -o 6 -p 18 -i 1 -d 29


echo "...set config valid"
./serial/write8fast 0x1D 1
echo "...set muxing_allowed"
./bin/v6test -s $slot -o 8 -p 13 -i 0 -d 1

#-------------------------------------------------------------
# Mux-specific parameters
#-------------------------------------------------------------

for index in {0..44}
do

echo "Set tston control bit to AUTO"
./bin/v6test -s $slot -o 6 -p 57 -i 0 -d 1

echo "Set tston to bit 32"
./bin/v6test -s $slot -o 2 -p 48 -i $index -d 0x00002005

echo "Set partbit to bit 33"
./bin/v6test -s $slot -o 2 -p 49 -i $index -d 0x00002105

echo "Set config valid bit"
./bin/v6test -s $slot -o 6 -p 0 -d 1

done

echo "Copy part present directly to status area"
./bin/v6test -s $slot -o 6 -p 14 -i 23 -d 1

#-----------------------------------------------
# Set debug output to serial
./bin/v6test -s $slot -o 6 -p 15 -i 7 -d 9
done

echo "Clear CRC counter"
bin/v6test -c -a 0x1A4 -d 8

