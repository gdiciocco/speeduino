# Pompa coolant EMP CAN — configurazione e logica di controllo

## 1. Scopo e campo di applicazione

Questo documento descrive il modulo di gestione della pompa elettrica EMP
integrato nella variante Caponord STM32 di Speeduino.

Sono documentati:

- i parametri configurabili dal menu TunerStudio `EMP Water Pump`;
- i parametri presenti nella pagina di configurazione ma non mostrati
  direttamente all'utente;
- le costanti interne non configurabili;
- gli ingressi usati dal regolatore;
- le formule del controllo closed-loop;
- la logica di after-run e Power Hold;
- stati, fault, flag, canali realtime e campi del logger;
- i vincoli di validità e una procedura consigliata di taratura.

L'impianto non usa una valvola termostatica. La pompa è quindi responsabile
sia della portata minima necessaria a evitare ristagni e punti caldi, sia della
regolazione della temperatura del liquido. Il target CLT rimane costante:
IAT, carico motore, velocità veicolo e ventola modificano l'azione di controllo,
non la temperatura obiettivo.

> **Avvertenza**
>
> I valori predefiniti sono una base prudenziale per lo sviluppo, non una
> calibrazione definitiva del circuito idraulico. Prima dell'uso su strada
> devono essere verificati portata minima, regime massimo ammesso dalla pompa,
> posizione del sensore CLT, capacità del radiatore e comportamento della
> pompa in caso di perdita dei messaggi CAN.

## 2. Architettura

Il modulo è diviso in due livelli:

1. `emp_pump.cpp` contiene la macchina a stati, il regolatore termico e la
   codifica/decodifica del protocollo CAN. Non dipende direttamente
   dall'hardware Speeduino.
2. `opf_core.cpp` abilita l'integrazione custom Caponord, legge la configurazione
   dalla pagina 15, valida i parametri, raccoglie gli ingressi ECU e usa CAN1
   per trasmettere e ricevere i frame della pompa.

Il controllo viene aggiornato a 10 Hz. Il comando CAN viene inviato subito
quando cambia e viene comunque ripetuto ogni 500 ms come heartbeat.

## 3. Inizializzazione e comportamento predefinito

La configurazione EMP usa:

| Elemento | Valore |
|---|---:|
| Magic di configurazione | `0xE6A5` |
| Versione configurazione | `2` |
| Flag predefiniti | `0x2E` |
| Pompa globalmente abilitata al primo avvio | No |
| Closed-loop predisposto | Sì |
| After-run predisposto | Sì |
| Power Hold predisposto | Sì |
| Funzionamento durante cranking | Sì |
| Hot boot recovery | No |

Quando magic o versione non corrispondono, `opf_core` carica tutti i default
descritti in questo documento. Il bit globale `Enable EMP pump` resta spento:
la pompa non viene attivata automaticamente dopo un aggiornamento firmware.

Dopo aver verificato la configurazione in TunerStudio è necessario abilitarla
esplicitamente e salvare/burnare la pagina.

## 4. Parametri esposti in TunerStudio

Tutti i parametri risiedono nella pagina 15, agli offset indicati. Le
temperature sono memorizzate internamente con offset `+40`, ma TunerStudio le
mostra nelle unità selezionate dal progetto.

### 4.1 Abilitazioni e comunicazione

| Nome TunerStudio | Simbolo INI | Offset | Range TS | Default | Descrizione |
|---|---|---:|---:|---:|---|
| Enable EMP pump | `empPumpEnabled` | 106 bit 0 | Off/On | Off | Abilitazione globale. Se disabilitato, il modulo porta il comando a zero e rilascia il Power Hold eventualmente attivo. |
| Enable after-run | `empPumpAfterRunEnabled` | 106 bit 1 | Off/On | On | Consente il raffreddamento dopo l'arresto motore. |
| Keep pump power alive | `empPumpPowerHoldEnabled` | 106 bit 2 | Off/On | On | Usa i comandi Power Hold previsti dal controller EMP. Non mantiene alimentata la ECU. |
| Run while cranking | `empPumpRunDuringCranking` | 106 bit 3 | Off/On | On | Considera il cranking come stato motore attivo. |
| Recover after ECU hot reboot | `empPumpHotBootRecovery` | 106 bit 4 | Off/On | Off | Permette di iniziare un after-run dopo un riavvio ECU se la CLT è già sopra la soglia di avvio. |
| Running control strategy | `empPumpClosedLoopEnabled` | 106 bit 5 | Curve fallback/Closed loop | Closed loop | Se spento, a motore acceso usa direttamente la curva CLT→RPM. |
| Controller address | `empPumpControllerAddress` | 107 | 0–240 | 150 (`0x96`) | Source address CAN della pompa, usato anche per riconoscere i frame di stato. |
| ECU source address | `empPumpSourceAddress` | 108 | 0–240 | 163 (`0xA3`) | Source address inserito nei frame di comando ECU→pompa. Deve essere diverso dall'indirizzo pompa. |
| Engine stop debounce | `empPumpStopDebounce` | 109 | 0–25,5 s | 1,0 s | Tempo per confermare l'arresto motore prima di passare a after-run o stop. |
| Status timeout | `empPumpStatusTimeoutSeconds` | 147 | 1–30 s | 3 s | Età massima ammessa dell'ultimo messaggio di stato principale mentre la pompa dovrebbe essere operativa. |

Gli indirizzi sono mostrati in decimale da TunerStudio. Con i default:

```text
ID comando = 0x18EF96A3
```

### 4.2 Limiti generali di velocità

| Nome TunerStudio | Simbolo INI | Offset | Range TS | Default | Descrizione |
|---|---|---:|---:|---:|---|
| Minimum running speed | `empPumpMinimumRunRpm` | 110–111 | 500–12000 RPM | 1500 RPM | Limite assoluto inferiore per ogni comando non nullo. |
| Maximum speed | `empPumpMaximumRpm` | 112–113 | 500–12000 RPM | 6000 RPM | Limite assoluto superiore del comando. |
| CLT sensor failsafe speed | `empPumpFailsafeRpm` | 116–117 | 500–12000 RPM | 3000 RPM | Comando usato con motore attivo quando la CLT non è valida. |
| Command ramp | `empPumpRampRpmPerSecond` | 118–119 | 0–12000 RPM/s | 2000 RPM/s | Limita la velocità di variazione del target trasmesso. Zero disabilita la rampa. |

Relazioni obbligatorie:

```text
0 < Minimum running speed <= Maximum speed <= 32767
Minimum running speed <= Failsafe speed <= Maximum speed
```

La rampa è applicata sia in salita sia in discesa quando la pompa è già in
rotazione. L'avvio da zero verso un comando valido è immediato; il comando di
arresto a zero è anch'esso immediato.

### 4.3 Target CLT e regolatore PI

| Nome TunerStudio | Simbolo INI | Offset | Range TS | Default | Effetto |
|---|---|---:|---:|---:|---|
| CLT target | `empPumpTargetTemperature` | 152 | -20–130 °C | 90 °C | Temperatura obiettivo costante del liquido. |
| Control deadband | `empPumpTemperatureDeadband` | 153 | 0–10 °C | 1 °C | Zona attorno al target nella quale l'errore usato da P e I vale zero. |
| Proportional gain | `empPumpProportionalGain` | 154–155 | 0–2000 RPM/°C | 250 | Correzione immediata per ogni grado di errore fuori deadband. |
| Integral gain | `empPumpIntegralGain` | 156–157 | 0–1000 RPM/(°C·s) | 12 | Velocità con cui viene eliminato l'errore termico persistente. |
| Integral correction limit | `empPumpIntegralLimitRpm` | 158–159 | 0–12000 RPM | 2000 RPM | Limite simmetrico positivo e negativo del contributo integrale. Deve essere ≤ Maximum speed. |

L'errore interno, dopo la deadband, è:

```text
se CLT > target + deadband:
    errore = CLT - target - deadband

se CLT < target - deadband:
    errore = CLT - target + deadband

altrimenti:
    errore = 0
```

La parte PI è:

```text
P = errore × Kp
I = I_precedente + errore × Ki × tempo
PI = P + I
```

L'integratore è limitato a `±Integral correction limit`. È inoltre presente
anti-windup condizionale:

- se l'uscita supera il massimo e l'errore vorrebbe aumentarla ancora,
  l'ultimo incremento integrale viene rifiutato;
- se l'uscita scende sotto la portata minima e l'errore vorrebbe ridurla
  ancora, l'ultimo decremento integrale viene rifiutato;
- l'integrale può comunque scaricarsi quando l'errore cambia direzione.

Per robustezza, il tempo attribuito a una singola iterazione integrale è
limitato internamente a 1 secondo.

### 4.4 Portata minima sicura in funzione del regime

La curva `EMP minimum safe flow by engine speed` contiene quattro punti:

| Punto | Engine RPM default | Minimum pump RPM default |
|---:|---:|---:|
| 1 | 0 | 1500 |
| 2 | 2000 | 1800 |
| 3 | 5000 | 2300 |
| 4 | 9000 | 3000 |

I simboli sono:

| Simbolo INI | Offset | Dimensione | Range TS |
|---|---:|---:|---:|
| `empPumpEngineRpmBins` | 174–181 | 4 × U16 | 0–20000 RPM |
| `empPumpMinimumFlowRpmBins` | 182–189 | 4 × U16 | 500–12000 RPM |

I bin del regime devono essere strettamente crescenti. Ogni valore di portata
deve essere compreso tra `Minimum running speed` e `Maximum speed`.

Il firmware interpola linearmente tra i punti. Sotto il primo bin usa il primo
valore; sopra l'ultimo usa l'ultimo. Il risultato viene comunque limitato tra
minimo e massimo assoluti.

Questa curva è un vincolo di sicurezza e non un target termico. Il PI e il
feed-forward possono aumentare la velocità sopra tale minimo, ma non possono
ridurla sotto di esso.

### 4.5 Feed-forward da carico motore

| Nome TunerStudio | Simbolo INI | Offset | Range TS | Default |
|---|---|---:|---:|---:|
| MAP x engine RPM feed-forward | `empPumpLoadFeedForwardGain` | 162–163 | 0–2000 RPM | 220 RPM |

La stima di carico termico usa RPM motore e MAP:

```text
RPM_limitati = min(engineRPM, 20000)
MAP_limitata = min(MAP, 250 kPa)

FF_carico =
    ((RPM_limitati × MAP_limitata) / 100)
    × LoadFeedForwardGain / 1000
```

Interpretazione pratica: con 1000 RPM e 100 kPa il contributo è uguale al
valore del parametro. Con il default:

```text
1000 RPM, 100 kPa -> 220 RPM pompa
5000 RPM, 100 kPa -> 1100 RPM pompa
5000 RPM, 150 kPa -> 1650 RPM pompa
```

Il feed-forward anticipa il calore prodotto dal motore senza aspettare che la
CLT salga. Non modifica il target CLT.

### 4.6 Compensazione IAT

| Nome TunerStudio | Simbolo INI | Offset | Range TS | Default |
|---|---|---:|---:|---:|
| IAT reference | `empPumpIatReferenceTemperature` | 164 | -40–100 °C | 20 °C |
| IAT compensation gain | `empPumpIatCompensationGain` | 165–166 | 0–500 RPM/°C | 18 RPM/°C |

Formula:

```text
FF_IAT = (IAT_filtrata - IAT_reference) × IAT_gain
```

Il termine può essere positivo o negativo:

- aria più calda del riferimento aumenta preventivamente il comando;
- aria più fredda lo riduce;
- il limite di portata minima impedisce comunque di scendere sotto una
  circolazione sicura.

La IAT è filtrata internamente con un IIR:

```text
IAT_filtrata += (IAT_nuova - IAT_filtrata) / 8
```

Il filtro viene aggiornato solo quando la IAT è valida. In caso di IAT non
valida il relativo contributo viene omesso e viene esposto il fault
`FAULT_IAT_INVALID`; il controllo CLT continua.

### 4.7 Compensazione del flusso d'aria: VSS e ventola

| Nome TunerStudio | Simbolo INI | Offset | Range TS | Default |
|---|---|---:|---:|---:|
| Full ram-air speed | `empPumpAirflowFullSpeedKph` | 167 | 1–255 km/h | 100 km/h |
| Ram-air RPM relief at full speed | `empPumpAirflowReliefRpm` | 168–169 | 0–12000 RPM | 600 RPM |
| Fan equivalent air speed | `empPumpFanEquivalentSpeedKph` | 170 | 0–255 km/h | 35 km/h |

Il flusso d'aria equivalente è:

```text
flusso = VSS, se il canale VSS è configurato e valido
flusso += FanEquivalentSpeed, se la ventola è accesa
flusso = min(flusso, FullRamAirSpeed)

FF_aria =
    flusso × AirflowReliefRpm / FullRamAirSpeed
```

`FF_aria` viene sottratto dal feed-forward totale. A velocità elevata o con
ventola attiva il radiatore ha maggiore capacità e serve meno anticipo di
portata; il PI resta responsabile di correggere eventuali differenze reali.

Se VSS non è configurata, il contributo della velocità veicolo è zero. La
ventola può ancora fornire il proprio contributo equivalente.

Il modulo non sovrascrive direttamente il controllo ventola Speeduino. Ne usa
lo stato come informazione sulla capacità di raffreddamento disponibile.

### 4.8 Anticipo sulla pendenza CLT

| Nome TunerStudio | Simbolo INI | Offset | Range TS | Default |
|---|---|---:|---:|---:|
| Positive CLT slope gain | `empPumpDerivativeGain` | 160–161 | 0–200 RPM/(°C/min) | 8 |

La pendenza viene campionata almeno ogni secondo:

```text
pendenza_grezza = ΔCLT × 60000 / Δtempo_ms
pendenza_grezza limitata tra -600 e +600 °C/min
pendenza_filtrata = (3 × precedente + nuova) / 4
```

Solo una pendenza positiva produce anticipo:

```text
FF_pendenza = max(pendenza_filtrata, 0) × DerivativeGain
```

Una CLT in discesa non genera un termine negativo: la riduzione del comando
viene gestita da feed-forward, PI e limite minimo.

### 4.9 Composizione finale del comando closed-loop

Il comando richiesto prima dei limiti è:

```text
FF_totale =
    FF_carico
  + FF_IAT
  - FF_aria
  + FF_pendenza

RPM_grezzi =
    RPM_minimi_dinamici
  + FF_totale
  + correzione_PI

RPM_richiesti =
    clamp(RPM_grezzi, RPM_minimi_dinamici, MaximumSpeed)
```

Successivamente viene applicata la rampa del comando e infine il clamp
assoluto `Minimum running speed` / `Maximum speed`.

### 4.10 Riconoscimento della capacità limite

| Nome TunerStudio | Simbolo INI | Offset | Range TS | Default |
|---|---|---:|---:|---:|
| Capacity-limited CLT delta | `empPumpCoolingLimitedDelta` | 171 | 1–30 °C | 3 °C |
| Thermal-overload CLT delta | `empPumpOverloadDelta` | 172 | 1–50 °C | 8 °C |
| Overload confirmation delay | `empPumpOverloadDelaySeconds` | 173 | 1–60 s | 5 s |

Lo stato `CapacityLimited` richiede contemporaneamente:

1. comando pompa al massimo;
2. errore reale `CLT - target` almeno pari a `Capacity-limited delta`;
3. capacità d'aria considerata esaurita.

La capacità d'aria è considerata esaurita quando si verifica almeno una delle
seguenti condizioni:

- VSS ≥ `Full ram-air speed`;
- ventola on/off accesa;
- ventola PWM al 100% (`fanDuty >= 200` nel formato interno Speeduino);
- controllo ventola disabilitato, quindi non esiste ulteriore attuatore da
  richiedere.

Se la condizione rimane continua, l'errore raggiunge `Thermal-overload delta`
e scade `Overload confirmation delay`, lo stato passa a `Overload`.

Il target non viene alzato per nascondere la saturazione. L'errore rimane
visibile nel logger e il firmware segnala esplicitamente il limite fisico del
sistema.

### 4.11 Curva CLT→RPM di after-run e fallback

La curva `EMP pump after-run / fallback` ha sei punti:

| CLT default | RPM pompa default |
|---:|---:|
| 40 °C | 1500 RPM |
| 70 °C | 1500 RPM |
| 85 °C | 2000 RPM |
| 95 °C | 3000 RPM |
| 105 °C | 4500 RPM |
| 115 °C | 6000 RPM |

| Simbolo INI | Offset | Dimensione |
|---|---:|---:|
| `empPumpTemperatureBins` | 126–131 | 6 × U08 |
| `empPumpRpmBins` | 132–143 | 6 × U16 |

I bin temperatura devono essere strettamente crescenti.

La curva viene usata:

- durante l'after-run;
- a motore acceso se `Running control strategy` è impostata su
  `Curve fallback`;
- durante il debounce di arresto motore quando è selezionato il fallback.

Durante l'after-run il risultato della curva non può scendere sotto
`Minimum after-run speed`.

### 4.12 Anti heat soak / after-run

| Nome TunerStudio | Simbolo INI | Offset | Range TS | Default |
|---|---|---:|---:|---:|
| Start temperature | `empPumpAfterRunStartTemperature` | 122 | -40–215 °C | 95 °C |
| Stop temperature | `empPumpAfterRunStopTemperature` | 123 | -40–215 °C | 85 °C |
| Maximum duration | `empPumpAfterRunMaximumSeconds` | 120–121 | 1–3600 s | 180 s |
| Minimum after-run speed | `empPumpAfterRunMinimumRpm` | 114–115 | 500–12000 RPM | 1800 RPM |
| Battery cut-off | `empPumpBatteryCutoff` | 124 | 8,0–16,0 V | 11,5 V |
| Battery restart threshold | `empPumpBatteryResume` | 125 | 8,0–16,0 V | 12,0 V |

Vincoli:

```text
Stop temperature < Start temperature
Minimum running speed <= Minimum after-run speed <= Maximum speed
Battery cut-off <= Battery restart threshold
Maximum duration > 0
```

Sequenza:

1. durante il funzionamento, se CLT ≥ `Start temperature`, il Power Hold viene
   armato quando abilitato;
2. dopo che l'arresto è rimasto valido per `Engine stop debounce`, viene
   aperta la finestra temporale di after-run;
3. l'after-run parte subito se la CLT è già sopra la soglia, oppure può partire
   successivamente se la CLT risale per heat soak durante la finestra, purché
   ECU e controller pompa siano ancora alimentati;
4. il comando è il massimo tra curva CLT→RPM e `Minimum after-run speed`;
5. l'after-run termina quando CLT ≤ `Stop temperature`, la CLT non è valida o
   scade la durata massima;
6. alla fine viene inviato il comando esplicito di rilascio Power Hold.

Protezione batteria:

- sotto `Battery cut-off` la pompa viene fermata e si memorizza il fault
  batteria;
- il timer di after-run continua a scorrere;
- la pompa può ripartire solo quando la tensione raggiunge
  `Battery restart threshold`.

`Hot boot recovery` viene valutato una sola volta dopo il boot. Se abilitato e
la CLT è già sopra la soglia, ricrea una finestra completa di after-run anche
se il firmware non ha osservato il motore acceso prima del riavvio.

> Il Power Hold mantiene attivo il controller della pompa secondo il protocollo
> EMP; non mantiene alimentata la ECU. Per controllare l'intero after-run,
> l'alimentazione della ECU deve rimanere disponibile dopo key-off.

### 4.13 Test di servizio

| Nome TunerStudio | Simbolo INI | Offset | Range TS | Default |
|---|---|---:|---:|---:|
| Test speed | `empPumpManualTestRpm` | 144–145 | 500–12000 RPM | 2000 RPM |
| Test duration | `empPumpManualTestSeconds` | 146 | 1–255 s | 10 s |

Pulsanti disponibili:

- `Start timed test`: avvia la pompa al regime configurato per la durata
  configurata;
- `Stop`: annulla il test;
- `Clear faults`: azzera i fault memorizzati e il contatore degli errori TX.

Il test è accettato solo se:

- configurazione valida;
- modulo globalmente abilitato;
- RPM test tra minimo e massimo;
- durata diversa da zero.

Lo stato di test ha priorità sul normale controllo motore fino alla scadenza o
al comando Stop.

## 5. Parametri presenti ma non esposti nei pannelli

Questi campi fanno parte della pagina di configurazione e sono descritti
nell'INI, ma non sono mostrati nei pannelli operativi:

| Campo | Offset | Default | Uso |
|---|---:|---:|---|
| `empPumpUnusedFlags` | 106 bit 6–7 | 0 | Bit riservati per funzioni future. |
| `empPumpConfigMagic` | 148–149 | `0xE6A5` | Identifica una configurazione EMP inizializzata. |
| `empPumpConfigVersion` | 150 | 2 | Forza il caricamento dei nuovi default quando cambia il layout o la semantica. |
| `empPumpReserved151` | 151 | 0 | Byte riservato. |
| `Unused15_225_255` | 225–255 | tutti zero | 31 byte riservati per futuri sviluppi; gli offset 190–209 appartengono all'Idle Advance e 210–224 all'autotune IAC closed-loop. |

Non devono essere modificati manualmente. Un magic o una versione errati
rendono la configurazione non valida e al successivo setup causano il
caricamento dei default correnti.

## 6. Costanti interne non configurabili

### 6.1 Bus e periodicità

| Costante | Valore | Note |
|---|---:|---|
| Bus usato | CAN1 esistente (`Can0` nel core STM32) | Condiviso con le altre funzioni Caponord. |
| Bit rate | 500 kbit/s | Non configurato dal modulo EMP. La pompa deve supportare questa velocità. |
| Periodo heartbeat | 500 ms | Frequenza minima comando: 2 Hz. |
| Frequenza aggiornamento regolatore | 10 Hz | Richiamato da `opf_core::runLoop`. |
| Tipo identificatore | Extended 29 bit | Obbligatorio per i frame EMP. |

### 6.2 Identificatori CAN

Con `nn = controllerAddress` e `ss = sourceAddress`:

| Messaggio | Formula ID |
|---|---|
| Comando ECU→pompa | `0x18EF0000 \| (nn << 8) \| ss` |
| Status 1 | `0x18FF0300 \| nn` |
| Status 2 | `0x18FF2300 \| nn` |
| Status 3 | `0x18FF2400 \| nn` |
| Temperatura esterna | `0x18FF4300 \| nn` |
| Address claim | `0x18EEFF00 \| nn` |

### 6.3 Byte di controllo

| Valore | Significato usato dal firmware |
|---:|---|
| `0xFC` | Pompa off, nessuna modifica Power Hold |
| `0xF0` | Pompa off, Power Hold off |
| `0xFD` | Marcia avanti, nessuna modifica Power Hold |
| `0xF1` | Marcia avanti, Power Hold off |
| `0xF5` | Marcia avanti, Power Hold on |

Il regime nel comando è codificato a `0,5 RPM/bit`:

```text
rawRPM = targetRPM × 2
```

Quando il target è zero, i due byte RPM sono trasmessi come `0xFFFF`.

### 6.4 Validità sensori

La CLT è considerata valida se:

```text
2 < cltADC < 1021
-40 °C <= CLT <= 215 °C
```

La IAT è considerata valida con gli stessi limiti:

```text
2 < iatADC < 1021
-40 °C <= IAT <= 215 °C
```

VSS è considerata disponibile quando `configPage2.vssMode != VSS_MODE_OFF`.
Non viene eseguito un controllo interno di plausibilità dinamica o di sensore
VSS bloccato.

### 6.5 Limiti interni di calcolo

| Grandezza | Limite interno |
|---|---:|
| MAP usata nel feed-forward | massimo 250 kPa |
| RPM motore usati nel feed-forward | massimo 20000 RPM |
| Pendenza CLT grezza | da -600 a +600 °C/min |
| Durata massima singolo passo integratore | 1000 ms |
| Diagnostiche feed-forward e PI | saturate nel formato S16 |
| Errore CLT realtime | saturato nel formato S8 |
| Cooling demand realtime | 0–100%, risoluzione 0,5% |

## 7. Ingressi non configurabili del regolatore

Gli ingressi sono raccolti automaticamente da `currentStatus`:

| Ingresso | Origine ECU | Uso |
|---|---|---|
| CLT | `currentStatus.coolant` | Variabile controllata, after-run e failsafe. |
| IAT | `currentStatus.IAT` | Stima temperatura ambiente durante il funzionamento. |
| RPM motore | `currentStatus.RPM` | Portata minima dinamica e feed-forward di carico. |
| MAP | `currentStatus.MAP` | Feed-forward di carico. |
| VSS | `currentStatus.vss` | Stima ram-air sul radiatore. |
| Tensione batteria | `currentStatus.battery10` | Protezione after-run. |
| Stato ventola | `currentStatus.fanOn` | Flusso equivalente e diagnostica capacità. |
| Duty ventola | `currentStatus.fanDuty` | Riconoscimento ventola PWM al 100%. |
| Stato rotazione | `currentStatus.rotationStatus` | Running, cranking o motore fermo. |

## 8. Stati operativi

### 8.1 Stato principale `capoEmpPumpState`

| Valore | Stato | Significato |
|---:|---|---|
| 0 | Disabled | Configurazione non valida o abilitazione globale spenta. |
| 1 | Stopped | Modulo valido e abilitato, motore fermo, nessun after-run attivo. |
| 2 | EngineActive | Motore running oppure cranking abilitato. |
| 3 | AfterRun | Raffreddamento post-arresto attivo. |
| 4 | ServiceTest | Test temporizzato attivo. |

### 8.2 Stato termico `capoEmpPumpThermalState`

| Valore | Stato | Significato |
|---:|---|---|
| 0 | Inactive | Regolatore termico inattivo. |
| 1 | Warmup | CLT sotto target/deadband e comando bloccato alla portata minima sicura. |
| 2 | ClosedLoop | Regolazione normale o fallback a curva. |
| 3 | CapacityLimited | Pompa e capacità d'aria esaurite, errore sopra il primo delta. |
| 4 | Overload | Capacità limitata, errore elevato e ritardo di conferma scaduto. |
| 5 | AfterRun | Strategia anti heat soak attiva. |
| 6 | Failsafe | CLT non valida con motore attivo; usato il regime failsafe. |
| 7 | Service | Test di servizio attivo. |

## 9. Flag closed-loop realtime

Il canale `capoEmpPumpControlFlags` è un byte:

| Bit | Canale derivato | Significato |
|---:|---|---|
| 0 | `capoEmpPumpClosedLoopActive` | Algoritmo closed-loop attivo. |
| 1 | `capoEmpPumpAtMinimum` | Comando calcolato limitato alla portata minima dinamica. |
| 2 | `capoEmpPumpAtMaximum` | Comando calcolato limitato al massimo. |
| 3 | `capoEmpPumpIatValid` | IAT valida e usata. |
| 4 | `capoEmpPumpVssValid` | VSS configurata/disponibile. |
| 5 | `capoEmpPumpFanOn` | Ventola attualmente attiva. |
| 6 | `capoEmpPumpPositiveSlope` | Pendenza CLT filtrata positiva. |
| 7 | `capoEmpPumpAirflowAtCapacity` | Controllo ventola già al massimo o non disponibile. |

## 10. Fault

Il canale `capoEmpPumpFaults` è una maschera U16:

| Bit | Simbolo | Tipo | Significato |
|---:|---|---|---|
| 0 | `FAULT_STATUS_TIMEOUT` | Dinamico | Nessuno status principale ricevuto entro il timeout mentre la pompa dovrebbe operare. |
| 1 | `FAULT_TX` | Memorizzato fino a TX riuscita/clear | Scrittura CAN fallita. |
| 2 | `FAULT_CLT_INVALID` | Memorizzato | CLT non valida durante uno stato operativo. |
| 3 | `FAULT_BATTERY_LOW` | Memorizzato | Batteria sotto cutoff durante after-run. |
| 4 | `FAULT_CONFIG` | Memorizzato/condizionale | Configurazione non valida. |
| 5 | `FAULT_SERVICE_REQUIRED` | Da status pompa | Controller segnala necessità di assistenza. |
| 6 | `FAULT_DERATED` | Da status pompa | Pompa in funzionamento derated. |
| 7 | `FAULT_NOT_OPERABLE` | Da status pompa | Pompa non operabile. |
| 8 | `FAULT_COMMAND_NOT_EXTERNAL` | Da status pompa | La sorgente comando non risulta esterna; nascosto quando il target è zero. |
| 9 | `FAULT_HVIL` | Da status pompa | Stato HVIL non valido/aperto secondo status 3. |
| 10 | `FAULT_IAT_INVALID` | Dinamico | IAT esclusa dal feed-forward. Il controllo CLT continua. |
| 11 | `FAULT_COOLING_LIMITED` | Dinamico | Sistema nelle condizioni `CapacityLimited`. |
| 12 | `FAULT_THERMAL_OVERLOAD` | Dinamico | Sistema nello stato `Overload`. |

`Clear faults` azzera fault memorizzati e contatore TX, ma un fault dinamico o
una condizione ancora presente ricompare immediatamente.

## 11. Capability rilevate dal protocollo

`capoEmpPumpCapabilities` indica quali tipi di messaggio sono stati osservati
dal boot:

| Bit | Canale | Messaggio rilevato |
|---:|---|---|
| 0 | `capoEmpPumpHasStatus1` | Status 1 |
| 1 | `capoEmpPumpHasStatus2` | Status 2 |
| 2 | `capoEmpPumpHasStatus3` | Status 3 |
| 3 | `capoEmpPumpHasExtTemp` | Temperatura esterna |
| 4 | `capoEmpPumpHasAddressClaim` | Address claim |

I bit sono informativi e restano impostati dopo la prima ricezione.

## 12. Canali realtime e logger

Il blocco realtime Caponord:

| Proprietà | Valore |
|---|---:|
| Offset iniziale | 176 |
| Magic realtime | `0xCA50` |
| Versione layout | 8 |
| Dimensione totale `ochBlockSize` | 320 byte (255-319 oltre i canali pompa: dash input a 255, 256-319 di riserva) |

### 12.1 Canali pompa e protocollo

| Canale | Offset | Tipo/scala | Descrizione |
|---|---:|---|---|
| `capoEmpPumpState` | 214 | U08 | Stato principale. |
| `capoEmpPumpCapabilities` | 215 | U08 bits | Messaggi CAN rilevati. |
| `capoEmpPumpFaults` | 216 | U16 bits | Maschera fault. |
| `capoEmpPumpTargetRpm` | 218 | U16 RPM | Target effettivamente trasmesso dopo rampa. |
| `capoEmpPumpActualRpm` | 220 | U16 RPM | Velocità riportata dalla pompa. |
| `capoEmpPumpActualPercent` | 222 | U08 × 0,5% | Percentuale riportata dalla pompa. |
| `capoEmpPumpControllerStatus` | 223 | U08 | Codice stato controller decodificato. |
| `capoEmpPumpStatusSummary` | 224 | U08 bits | Sintesi direzione/sorgente/service/operation. |
| `capoEmpPumpVoltage` | 225 | U16 × 0,05 V | Tensione riportata. |
| `capoEmpPumpCurrent` | 227 | U16 × 0,05 A − 1600 A | Corrente riportata secondo scala protocollo. |
| `capoEmpPumpPower` | 229 | U16 W | Potenza riportata. |
| `capoEmpPumpExternalTemp` | 231 | U16 × 0,03125 − 273 °C | Temperatura esterna/controller. |
| `capoEmpPumpStatusAgeMs` | 233 | U16 ms | Età status principale; 65535 se mai ricevuto. |
| `capoEmpPumpAfterRunRemaining` | 235 | U16 s | Tempo after-run residuo. |
| `capoEmpPumpTxFailureCount` | 237 | U08 | Conteggio saturato degli errori TX. |
| `capoEmpPumpLastControl` | 238 | U08 hex | Ultimo byte di controllo inviato. |

### 12.2 Canali del regolatore

| Canale | Offset | Tipo/scala | Descrizione |
|---|---:|---|---|
| `capoEmpPumpThermalState` | 239 | U08 | Stato termico. |
| `capoEmpPumpControlFlags` | 240 | U08 bits | Flag closed-loop. |
| `capoEmpPumpCltTarget` | 241 | U08 temperatura | Target CLT configurato. |
| `capoEmpPumpFilteredIat` | 242 | U08 temperatura | IAT filtrata realmente usata. |
| `capoEmpPumpCltError` | 243 | S08 temperatura | Errore reale `CLT - target`, prima della deadband. |
| `capoEmpPumpCltSlope` | 244 | S16 °C/min | Pendenza filtrata CLT. |
| `capoEmpPumpMinimumFlowRpm` | 246 | U16 RPM | Portata minima interpolata corrente. |
| `capoEmpPumpFeedForwardRpm` | 248 | S16 RPM | Somma dei quattro termini feed-forward. |
| `capoEmpPumpPiCorrectionRpm` | 250 | S16 RPM | Correzione P + I corrente. |
| `capoEmpPumpCoolingDemand` | 252 | U08 × 0,5% | Posizione del comando tra minimo dinamico e massimo. |
| `capoEmpPumpSaturationSeconds` | 253 | U16 s | Durata continua della condizione CapacityLimited. |

Tutti questi canali sono inclusi nel logger TunerStudio. Il logger standard
continua inoltre a registrare CLT, IAT, MAP, RPM, VSS, duty ventola e gli altri
canali ECU utili alla correlazione.

## 13. Validazione completa della configurazione

La configurazione è accettata solo se tutte le condizioni seguenti sono vere:

```text
magic == 0xE6A5
version == 2

controllerAddress <= 0xF0
sourceAddress <= 0xF0
controllerAddress != sourceAddress

minimumRunRpm > 0
maximumRpm >= minimumRunRpm
maximumRpm <= 32767

minimumRunRpm <= afterRunMinimumRpm <= maximumRpm
minimumRunRpm <= failsafeRpm <= maximumRpm

afterRunStopTemperature < afterRunStartTemperature
afterRunMaximumSeconds > 0
batteryCutoff <= batteryResume
statusTimeoutSeconds > 0

-20 <= targetTemperature <= 130
temperatureDeadband <= 10
proportionalGain <= 2000
integralGain <= 1000
integralLimitRpm <= maximumRpm
derivativeGain <= 200
loadFeedForwardGain <= 2000
iatCompensationGain <= 500

airflowFullSpeedKph > 0
airflowReliefRpm <= maximumRpm
coolingLimitedDelta > 0
overloadDelta >= coolingLimitedDelta
overloadDelaySeconds > 0

i 6 bin CLT sono strettamente crescenti
i 4 bin RPM motore sono strettamente crescenti
ogni minimumFlowRpmBin è compreso tra minimumRunRpm e maximumRpm
```

Una configurazione non valida impedisce l'attivazione della pompa e imposta
`FAULT_CONFIG`.

## 14. Procedura consigliata di taratura

### Fase 1 — Verifica elettrica e CAN

1. Lasciare `Enable EMP pump` disabilitato.
2. Verificare che la pompa sia configurata per CAN a 500 kbit/s.
3. Confermare indirizzo pompa e indirizzo ECU.
4. Abilitare il modulo e usare un test temporizzato breve.
5. Controllare:
   - target e velocità misurata;
   - status age;
   - tensione, corrente e potenza;
   - controller status e fault;
   - rilascio corretto del Power Hold allo stop.

### Fase 2 — Determinazione della portata minima

1. Impostare temporaneamente guadagni e feed-forward conservativi.
2. Stabilire il minimo fisicamente affidabile della pompa.
3. Verificare la circolazione a minimo, medio e alto regime motore.
4. Compilare la curva `Minimum safe circulation` evitando valori che
   permettano ristagni nella testa o cavitazione.
5. Non usare il PI per compensare una curva minima insufficiente.

### Fase 3 — Feed-forward di carico

1. Portare IAT gain, airflow relief e derivative gain temporaneamente a zero.
2. Registrare transitori di carico con PI moderato.
3. Aumentare `MAP x engine RPM feed-forward` finché il comando anticipa il
   carico senza creare un eccessivo calo CLT.
4. Controllare separatamente basso regime/alto MAP e alto regime/MAP medio.

### Fase 4 — PI

1. Partire con Ki basso o nullo.
2. Aumentare Kp finché la CLT reagisce con sufficiente prontezza senza
   oscillazioni evidenti.
3. Aggiungere Ki lentamente per eliminare l'errore persistente.
4. Limitare l'integrale a quanto serve realmente per compensare errori
   stazionari.
5. Mantenere inizialmente una deadband di 1–2 °C.

Nel log confrontare:

```text
CLT
EMP CLT Target
EMP CLT Error
EMP Minimum Flow RPM
EMP Feed-forward RPM
EMP PI Correction RPM
EMP Pump Target RPM
EMP Pump Actual RPM
EMP Cooling Demand
```

### Fase 5 — IAT, VSS e ventola

1. Tarare IAT gain confrontando prove simili in giornate con temperatura
   ambiente diversa.
2. Tarare `Full ram-air speed` osservando la velocità oltre la quale ulteriore
   VSS modifica poco il comportamento termico.
3. Tarare `Ram-air RPM relief` con gradualità: un valore troppo alto porta il
   comando spesso al minimo e lascia tutto il lavoro correttivo al PI.
4. Tarare `Fan equivalent air speed` confrontando prove da fermo con ventola
   spenta e accesa.

### Fase 6 — Pendenza e protezioni

1. Aggiungere un derivative gain piccolo.
2. Verificare che aumenti la pompa durante una salita CLT reale, senza reagire
   eccessivamente ai singoli scatti di 1 °C del sensore.
3. Impostare `Capacity-limited delta` come primo scostamento chiaramente
   anomalo ma ancora gestibile.
4. Impostare `Thermal-overload delta` al limite oltre il quale sono richieste
   protezioni motore o riduzione del carico.
5. Usare un ritardo sufficiente a non classificare come overload un breve
   transitorio di comando.

### Fase 7 — After-run

1. Verificare che ECU e pompa restino alimentate per la durata desiderata.
2. Registrare il picco CLT dopo key-off.
3. Impostare la soglia start sotto il picco che si vuole prevenire.
4. Impostare la soglia stop con isteresi sufficiente.
5. Usare la minima velocità che garantisca miscelazione senza assorbimento
   inutile.
6. Verificare cutoff e resume batteria con alimentatore controllato o prova
   dedicata.

## 15. Limiti e aspetti da verificare sul veicolo

- Il modulo assume che la IAT sia rappresentativa dell'aria ambiente mentre
  il motore è acceso.
- Non è presente un controllo di plausibilità incrociato IAT/CLT né una
  diagnosi di VSS bloccata.
- Il feed-forward usa MAP×RPM come proxy del carico termico; non usa
  direttamente portata carburante o energia combustibile.
- La pendenza CLT è calcolata da una temperatura intera e può quindi procedere
  a gradini; il filtro interno riduce ma non elimina questo effetto.
- `Cooling demand` rappresenta l'uso della gamma di comando pompa, non una
  misura diretta della capacità termica residua del radiatore.
- Lo stato ventola viene osservato, ma il modulo EMP non modifica direttamente
  setpoint o tabella della ventola Speeduino.
- La temperatura esterna trasmessa dalla pompa è diagnostica e non viene usata
  come sostituto della IAT.
- Il comportamento autonomo della pompa quando il heartbeat CAN viene perso
  dipende dalla calibrazione interna del controller EMP e deve essere
  verificato fisicamente. In assenza di termostato è raccomandato un default
  autonomo non nullo e sicuro.
- La posizione del sensore CLT può non rappresentare immediatamente eventuali
  punti caldi locali nella testa; per questo la portata minima dinamica non
  deve essere ridotta solo perché la CLT misurata è sotto target.

## 16. Riferimenti nel codice

- `speeduino/emp_pump.h`: strutture, flag, stati, fault e interfaccia pubblica.
- `speeduino/emp_pump.cpp`: macchina a stati, regolatore e protocollo CAN.
- `speeduino/opf_core.cpp`: default, validazione, ingressi ECU e bridge CAN1.
- `speeduino/config_pages.h`: layout binario pagina 15.
- `speeduino/logger.cpp`: serializzazione del blocco realtime Caponord.
- `reference/speeduino.ini`: campi, pannelli, curve e logger TunerStudio.
- `test/test_emp_pump/test_emp_pump.cpp`: test del protocollo e del controllo.
