sh as.sh stub0.asm
sh dump.sh -b0 stub0.rom > stub.X
sh as.sh stub.asm
sh dump.sh -b7100 stub.rom >> stub.X