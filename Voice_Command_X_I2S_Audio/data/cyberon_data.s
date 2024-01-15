.pushsection command_data, "ax", %progbits
.incbin "data/SmartClockCmd_pack_WithTxt.bin"
.popsection

.pushsection license_data, "ax", %progbits
.incbin "data/LicensePsoc062S2.bin"
.popsection
