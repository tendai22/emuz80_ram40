uart_d  equ 01h
uart_c  equ 00h
        org 0
        ld  sp,01000h
rxloop: in  a,(uart_c)  ; PIR9
        and a,1         ; check RXIF
        jr  z,rxloop
        in  a,(uart_d)
        push af
txloop: in  a,(uart_c)  ; U3FIFO
        and a,2      ; check TXIF
        jr  z,txloop
        pop af
        out  (uart_d),a
        jr  rxloop
        end

