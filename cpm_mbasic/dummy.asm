;
;
;
const  equ 7f06h
conin  equ 7f09h
conout equ 7f0ch
        org 100h
loop:   call const
        and a 
        jr  z,loop
        call conin
        call conout
        jr  z,loop
        end
