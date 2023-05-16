bios    equ 7f00h
bdos    equ bios-0e00h
wboot   equ 7f03h
        org 0
        jp  wboot
        dw  0
        jp  bdos
        org 80H
        dw  0,0,0,0
        end
