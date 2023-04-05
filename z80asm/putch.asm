uart_d  equ 01h
uart_c  equ 00h
        org 0
        ld  sp,01000h
loop:
        ld  a,0x41
        push af
txloop: in  a,(uart_c)  ; U3FIFO
        and a,2      ; check TXIF
        jr  z,txloop
        pop af
        out  (uart_d),a
        jr  loop
        end

