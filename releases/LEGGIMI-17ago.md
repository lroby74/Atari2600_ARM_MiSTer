# RBF di consegna — 17 agosto 2026 (finale)

**Da usare: `Atari2600.rbf`** = `Atari2600_ARM_64MHz_seed9_17ago.rbf`
MD5 `50a470a743d228a350ca22a10ebadd76` — 3.965.000 byte.

```
setup  Slow 1100mV 100C .... +0,514 ns
setup  Slow 1100mV -40C .... +0,367 ns
critical warning 332148 .... 0
errori ..................... 0
```

Seed **9**, scelto fra sei con lo sweep deterministico:

```
seed   Slow 100C   Slow -40C   CW
  9      +0,514      +0,367     0    <<< CONSEGNATO
  3      +0,348      +0,155     0
 12      +0,278      +0,125     0
 20      +0,320      +0,117     0
 15      +0,078      -0,130     1
  5      +0,060      -0,155     1
```

Criterio: zero critical warning, **entrambi** i corner positivi, e fra quelli
che restano il **piu' alto minimo fra i due corner** — si massimizza il margine
peggiore, che e' il numero che decide se il fit regge su un'altra macchina.
Rispetto al seed 12 il margine peggiore quasi triplica (+0,367 contro +0,125).

Gli RBF precedenti della giornata restano nella cartella come rete di
sicurezza; il comportamento e' lo stesso, cambia solo il margine di
temporizzazione (e i primi tre non hanno le correzioni della sera).

Il dominio piu' stretto e' `pll_hdmi` in entrambi i corner — cioe' **quello che
non chiudeva** (era −0,078 col seed 7) ora chiude con mezzo nanosecondo di
margine. Tutti gli altri domini stanno sopra. Anche l'hold e' positivo ovunque
(minimo +0,061).

## Com'e' stato costruito, e perche' lo puoi rifare identico

Compilato dal **repo com'e'**, non da una sandbox: `bash tools/consegna_rbf.sh 9`.
I numeri sono venuti **identici** a quelli dello sweep fatto in una directory
diversa — cioe' il determinismo introdotto con `NUM_PARALLEL_PROCESSORS 1`
funziona davvero, non solo sulla carta.

Per riottenerlo servono:
- **Quartus 17.0.2 Lite Edition** (un'altra versione da' altri risultati);
- l'albero **pulito**, senza `db/`, `incremental_db/`, `output_files/`;
- il `.qsf` com'e' nel repo: **SEED 9** e **NUM_PARALLEL_PROCESSORS 1**.

> Nota: con `NUM_PARALLEL_PROCESSORS 1` la compilazione e' piu' lenta (il fitter
> qui ha impiegato ~14 minuti). E' il prezzo della riproducibilita': rimettendo
> `ALL` il core e' identico ma i numeri di timing cambiano da macchina a macchina.

## Cosa c'e' dentro rispetto all'ultimo RBF che avevi provato

```
ARM a 64,431819 MHz          (era 71,590910)  -> slack ARM +1,135 invece di centesimi
QuadTari                     invisibile ai giochi quando la voce OSD e' su Off
SEED                         9 (era 7)
NUM_PARALLEL_PROCESSORS      1  (era ALL)
SNAC ADC                     TOLTO su tua richiesta (sta in backup_speed/snacadc_v2_15ago/)
```

Invariati e gia' approvati: passo ARSP (sparo di Spiders + Lode Runner),
architettura ALFA, schermata di avvio.

## Il collaudo fatto in casa

- **Regressione sui 38 titoli, 64,43 contro 71,59**: 36 identici pixel per
  pixel, 0 bloccati. I due diversi sono **GridLock** e **Rightris**, e in
  nessuno dei due c'e' corruzione: l'ARM esegue le stesse identiche istruzioni,
  finisce solo in un momento diverso rispetto al 6507.
- **Su GridLock l'oracolo ha dato ragione al 64,43**: Gopher2600 sulla
  cartuccia vera scrive in COLUPF `0x90` e `0x40`, che sono i colori prodotti a
  64,43; quelli del 71,59 (`0xB2`, `0x62`) non li scrive mai.
- Su **Rightris** la differenza sono 84 pixel su 72.000 in una striscia alta 5
  righe in fondo, fra due gradazioni adiacenti dello stesso gradiente che il
  riferimento scrive entrambe: non e' decidibile dalla traccia dei registri e
  non e' un guasto.
- Le sonde di guasto (`[perso]`, scritture indicizzate scartate) danno **numeri
  identici sui due clock**: nessun peggioramento.
