;
;
;
bios    equ 7f00h
bdos    equ bios-0e00h

uart_d  equ 01h
uart_c  equ 00h

        org bdos
        ld  hl,0022h
        ret
        
        org bios
        jp  boot
        jp  wboot
        jp  const
        jp  conin
        jp  conout
        jp  list
        jp  punch
        jp  reader
        jp  home
        jp  seldsk
        jp  settrk
        jp  setsec
        jp  setdma
        jp  read
        jp  write
        jp  listst
        jp  sectran


boot:   ld  a,'0'
        jr  dohalt

wboot:  ld  sp, 7f00h
        jp  0100h

const:  in  a,(uart_c)
        and a,1         ; check RXRDY
        ret z           ; ret if zero, not-ready
        ld  a,0xff
        ret             ; ret with FFh

conin:  in  a,(uart_c)
        and a,1         ; check RXRDY
        jr  z,conin
        in  a,(uart_d)
        and a,7fh       ; clear MSB and set parity flag 
        ret
conout:  push af
putch1: in  a,(uart_c)
        and a,2         ; check TXRDY
        jr  z,putch1
        pop af
        out  (uart_d),a
        ret

list:   ld  a,'6'
        jr  dohalt
punch:  ld  a,'7'
        jr  dohalt
reader: ld  a,'8'
        jr  dohalt
home:   ld  a,'9'
        jr  dohalt
seldsk: ld  a,'A'
        jr  dohalt
settrk: ld  a,'B'
        jr  dohalt
setsec: ld  a,'C'
        jr  dohalt
setdma: ld  a,'D'
        jr  dohalt
read:   ld  a,'E'
        jr  dohalt
write:  ld  a,'F'
        jr  dohalt
listst: ld  a,'G'
        jr  dohalt
sectran:
        ld  a,'H'
dohalt: call conout
        ld  a,'-'
        call conout
        ld  a,'H'
        call conout
        ld  a,'A'
        call conout
        ld  a,'L'
        call conout
        ld  a,'T'
        call conout
        halt
        end



