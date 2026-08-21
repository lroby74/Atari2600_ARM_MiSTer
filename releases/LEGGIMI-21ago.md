# RBF di consegna — 21 agosto 2026

**Da usare: `Atari 2600 ARM_20260821.rbf`**
MD5 `16beae870d5d8ec31b57bcb41ec5d6e8` — 3.895.452 byte.

```
setup  Slow 1100mV 100C .... +0,553 ns
setup  Slow 1100mV -40C .... +0,326 ns
critical warning 332148 .... 0
errori ..................... 0
ALM ........................ 25.654 / 41.910 (61%)
M10K ....................... 372 / 553 (67%)
```

Il nome porta la data della compilazione: il MiSTer carica quello con la
data piu' recente, quindi le versioni vecchie si possono lasciare nella
cartella `_Console/` come rete di sicurezza. Il file `Atari 2600 ARM.mgl`
punta al nome **senza** data, ed e' cosi' che deve stare.

Seed **3**. Compilato con `db/` e `incremental_db/` cancellati, cioe' partendo
dal piazzamento vuoto come fa chi clona il repo.

## Che cosa cambia

### 1. Mapper FA2 — Star Castle Arcade adesso parte

Segnalazione dal forum MiSTer. Mancava del tutto il mapper **FA2** (CBS RAM
Plus esteso, schede Harmony/Melody): 7 banchi da 4K, 256 byte di RAM, e nelle
immagini da 32K un'intestazione di 1K di codice ARM da saltare. Scritto
seguendo **due riferimenti indipendenti** che concordano, Stella
(`CartFA2.cxx`) e Gopher2600 (`mapper_fa2.go`).

Il riconoscimento automatico segue Stella: 24K e 28K sono FA2 senza altre
prove; a 32K si entra solo se gli ultimi 3K sono a zero e non c'e' la firma
superchip. Provato su **925 ROM**: tre sole corrispondenze, tutte copie di
Star Castle.

### 2. SUPERbank (SB) riconosciuto per taglia

Il mapper c'era gia', mancava il riconoscimento: si entrava solo se
nell'immagine compariva `BD 00 08` o `AD 00 08`, e senza quel motivo una
cartuccia da 128K o 256K finiva sul mapper sbagliato. Ora 128K e 256K ricadono
su SB come fa Stella. Il CDF resta provato per primo, ed e' cio' che tiene al
sicuro Turbo Arcade, che e' CDFJ+ da 128K.

### 3. >>> Stabilize Video: corretto il DOPPIO VSYNC <<<

E' la correzione piu' importante di questa versione.

Quando la lunghezza del fotogramma cambiava di **1, 2 o 3 righe**, lo
stabilizzatore accendeva `vsync_override` ed emetteva un VSYNC proprio, ma
**non azzerava il contatore di riga**: poco dopo arrivava il VSYNC vero del
gioco e ne usciva **un secondo**. Misurato sul gameplay di Umi Machines:

|                              | prima      | dopo               |
|---|---|---|
| fronti di VBLANK per fotogramma | 0..1    | **1**              |
| righe attive                    | 0..240  | **240 fisse**      |
| lunghezza fotogramma in uscita  | **4..271** | **267..272** (= il gioco) |

Fotogrammi da **quattro righe con zero righe attive**: e' il ballerio per cui
i titoli DPC+ (i due *Umi*, *Epic Adventure V2.2*, *Evil Magician Returns*)
andavano tenuti con la voce OSD su `Off`.

Il difetto aveva **due facce**, dallo stesso ramo:

* i giochi che oscillano di continuo prendevano il doppio VSYNC;
* quelli che oscillano **una volta sola** restavano **sfasati per sempre**,
  perche' `vsync_override` si agganciava e non si spegneva piu': da li' in poi
  il VSYNC del gioco veniva **ignorato** e l'uscita partiva una o due righe in
  anticipo. *Neon Run* aveva 238 righe attive invece di 239 esattamente per
  questo.

*Star Castle* ha 270 righe fisse, non entrava mai in quel ramo, ed e' il
motivo per cui voleva `On` mentre gli altri volevano `Off`.

**Adesso la voce va bene su `On` per tutti, che e' il default**: appena si
carica il core non si tocca niente. Collaudato su hardware reale su una
quindicina di titoli.

### 4. README

Nuova sezione **Video options**, una nota sui controller a **due pulsanti**
(*1942 VCS* risponde al secondo pulsante su questo core e non sull'originale
`Atari7800_MiSTer`, verificato sui due core con la stessa ROM), i titoli SB
reali, e la sezione **Credits** con il credito a Claude Code e quella
sull'uso dell'IA.

### 5. Pulizia dei commenti

Tolti da tutti i sorgenti pubblicati i `TODO`, i `FIXME`, la parola "stub" e i
rimandi a file che non fanno parte del repo. Due erano **sbagliati**:
l'intestazione di `dpcplus_bridge.sv` dichiarava non implementati il
generatore pseudocasuale, il motore musicale e le CALLFUNCTION 254/255, che
sono tutti implementati piu' sotto nello stesso file; e un `FIXME` nel mapper
E7 chiedeva di aggiungere il banco di RAM superiore, che c'e' gia'. Nessuna
riga di logica e' stata toccata.

## Regressione

**37 titoli su 40 identici pixel per pixel**, cicli CPU identici in tutti e 40.

I tre diversi — *Evil Magician Returns*, *Neon Run*, *Tutankham* — sono
esattamente quelli che variano la lunghezza del fotogramma di 1..3 righe,
cioe' il solo caso che la correzione tocca, e in tutti e tre il VSYNC in
uscita ora coincide con quello del gioco invece di partire in anticipo su un
valore rimasto agganciato. Su Tutankham, che produce fotogrammi da **256 righe
esatte e costanti**, in uscita ne davamo 255, 256 o 257: adesso 256 fissi.

## La scelta del seed

Campagna su 12 seed, poi **giro di verifica dei migliori sulla netlist
definitiva** — e li' la classifica si e' rimescolata: il seed 4, primo nella
campagna larga con +0,383 ns, e' sceso a +0,252, mentre il 3 e l'8 sono saliti.
Un solo registro di differenza basta a spostare il fitter di oltre 0,1 ns, e un
seed scelto su una netlist diversa da quella che si consegna e' un numero
ereditato, non misurato.

```
seed   Slow 100C   Slow -40C   PEGGIORE   CW
  3      +0,553      +0,326     +0,326     0   <<< CONSEGNATO
  8      +0,649      +0,303     +0,303     0
  4      +0,417      +0,252     +0,252     0
  6      +0,256      +0,192     +0,192     0
 11      +0,445      +0,100     +0,100     0
 13      +0,208      +0,087     +0,087     0
```

Criterio: zero critical warning, **entrambi** i corner positivi, e fra quelli
che restano il **piu' alto minimo fra i due corner** — si massimizza il margine
peggiore, che e' il numero che decide se il fit regge su un'altra macchina.
Fra il 3 e l'8 ci sono 0,023 ns, cioe' rumore del fitter: sono equivalenti.

## Un tentativo bocciato, per chi ripercorre la strada

Prima di trovare il doppio VSYNC avevo scritto un **aggancio della cadenza
verticale** che forzava il fotogramma a lunghezza costante. Sul banco sembrava
perfetto. **Sul ferro l'immagine scorreva dall'alto verso il basso a velocita'
costante**: il fotogramma usciva da 272 righe mentre il gioco ne disegnava 267,
e il contenuto slittava di 5 righe ogni volta. E' stato ritirato. La causa vera
era altrove e valeva quattro righe.

## Nota sulle dimensioni delle ROM

I giochi **senza ARM** stanno nella SDRAM del MiSTer, non nell'FPGA, e arrivano
a **256 KB** (il limite sono i 19 bit di indirizzo, non la memoria). Le
cartucce **ARM** (CDF, CDFJ, CDFJ+, DPC+) vengono invece **copiate nei blocchi
M10K** e si fermano a **128 KB**. Non risulta esistere una ROM ARM piu' grande,
quindi il limite non e' stato alzato: costerebbe margine di timing per una
capacita' che nessun gioco chiede.
