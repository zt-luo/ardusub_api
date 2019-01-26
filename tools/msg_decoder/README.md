# Usage  

``` powershell
.\decoder.exe -h

Usage:
  decoder.exe [OPTION…] - decode MAVLink msg frome json file or raw hex data

Help Options:
  -h, --help                                     Show help options
  --help-all                                     Show all help options

Application Options:
  -j, --json=<path to MAVLink msg json file>     Decode from json file
  -r, --raw=<raw hex separated by spaces>        Decode from raw hex
```
# Example  

``` powershell
.\decoder.exe -r fd0b000000ff0045000003000300f5010300000001eb6f fd0b000002ff0045000003000300f50103000000013496 fd0900000fff00000000000000000608c00403db85

.\decoder.exe -j .\msg.json > .\msg.txt
```

json file is export from Wireshark.  
File -> Export Packet Dissections -> As JSON...

![Export Packet Dissections](https://raw.githubusercontent.com/luozongtong123/ardusub_api/master/docs/md_img/export_packet_dissections.png)

details:TBD
