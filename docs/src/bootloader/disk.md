# Disk


## BIOS disk read
`AH = 01`

The following status codes represent controller status after last disk operation:
|Status (AL)|Description|
|-|-|
|00|no error|
|01|bad command passed to driver|
|02|address mark not found or bad sector|
|03|diskette write protect error|
|04|sector not found|
|05|fixed disk reset failed|
|06|diskette changed or removed|
|07|bad fixed disk parameter table|
|08|DMA overrun|
|09|DMA access across 64k boundary|
|0A|bad fixed disk sector flag|
|0B|bad fixed disk cylinder|
|0C|unsupported track/invalid media|
|0D|invalid number of sectors on fixed disk format|
|0E|fixed disk controlled data address mark detected|
|0F|fixed disk DMA arbitration level out of range|
|10|ECC/CRC error on disk read|
|11|recoverable fixed disk data error, data fixed by ECC|
|20|controller error (NEC for floppies)|
|40|seek failure|
|80|time out, drive not ready|
|AA|fixed disk drive not ready|
|BB|fixed disk undefined error|
|CC|fixed disk write fault on selected drive|
|E0|fixed disk status error/Error reg = 0|
|FF|sense operation failed|