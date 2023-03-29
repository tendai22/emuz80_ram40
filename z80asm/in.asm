uart_d  equ 01h
uart_c  equ 00h
        org 0
loop:   in  a,(uart_c)
        jr  loop
        end
