#!/usr/bin/env python3
"""
Simulatore Modbus TCP per openHAB - pymodbus 3.12

openhabian@openhabian:/etc/openhab/simulatore_modbus $ python3 modbus_simulator.py

==================================================
192.168.188.54:5020  ->  termo_bridge  (unit_id=1)
192.168.188.54:5021  ->  pdc_bridge    (unit_id=1)

Logica PLC simulata:
  FReleThtN (FeedBRele):
    - scrittura 1 su OnThtN  -> dopo 10s: FReleThtN = 1
    - scrittura 1 su OffThtN -> dopo 10s: FReleThtN = 0

  ReleThtN (StatoRele):
    - ReleThtN = 1  se  SetpointN > TemperaturaN
    - ReleThtN = 0  altrimenti
    - ricalcolato ogni volta che cambia temperatura o setpoint

Things openHAB:
  Bridge modbus:tcp:termo_bridge [ host="192.168.188.54", port=5020, id=1 ]
  Bridge modbus:tcp:pdc_bridge   [ host="192.168.188.54", port=5021, id=1 ]
"""

import asyncio
import threading
import time
import random
import logging

from pymodbus.datastore import (
    ModbusServerContext,
    ModbusDeviceContext,
    ModbusSequentialDataBlock,
)
from pymodbus.server import StartAsyncTcpServer

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
)
log = logging.getLogger("modbus_sim")


# ============================================================
#  Valori iniziali - 13 zone
# ============================================================
ZONE_NOMI = [
    "Biliardo PI",       # Tht1
    "Antibagno PI",      # Tht2
    "App.arancione PT",  # Tht3
    "AngoloTV",          # Tht4
    "Antibagno PT",      # Tht5
    "Soggiorno",         # Tht6
    "Cucina",            # Tht7
    "App.rosso",         # Tht8
    "App.verde",         # Tht9
    "Corridoio",         # Tht10
    "Arianna",           # Tht11
    "Giolo",             # Tht12
    "Eleonora",          # Tht13
]

TEMP_C  = [20, 19, 20, 21, 18, 22, 21, 20, 19, 17, 21, 20, 21]
UMID_PC = [55, 60, 52, 50, 62, 48, 50, 53, 55, 58, 51, 52, 50]
SETP_C  = [21, 21, 22, 22, 21, 22, 22, 21, 21, 20, 22, 22, 22]
# FReleTht iniziale: 1=acceso, 0=spento
FRELE   = [ 1,  1,  1,  1,  0,  1,  1,  1,  1,  0,  1,  1,  1]

def d10(v): return v * 10

# Indirizzi holding temperatura/umidita (poller04)
ADDR_T = [16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 54, 56, 58]
ADDR_U = [17, 21, 25, 29, 33, 37, 41, 45, 49, 53, 55, 57, 59]
# Indirizzi holding setpoint (poller08, 101..113)
ADDR_S = list(range(101, 114))

# Base coil per ogni termostato (0-based index)
# layout: +0=Test +1=All +2=Rele(StatoRele) +3=FRele(FeedBRele) +4=On +5=Off
COIL_BASE = [79 + n * 6 for n in range(10)] + [165, 171, 177]

FRELE_DELAY = 3   # secondi di ritardo prima di aggiornare FReleTht


# ============================================================
#  Helper blocco sequenziale
# ============================================================
def make_block(values: dict) -> ModbusSequentialDataBlock:
    """
    Crea un blocco sequenziale da indirizzo 0 fino a 65535.
    NOTA: ModbusDeviceContext applica offset -1 internamente:
      getValues(addr) legge data[addr+1]
    Quindi scriviamo ogni valore in data[addr+1].
    """
    data = [0] * 65536
    for addr, val in values.items():
        data[addr + 1] = val
    return ModbusSequentialDataBlock(0, data)


# ============================================================
#  TermoDeviceContext
# ============================================================
class TermoDeviceContext(ModbusDeviceContext):
    """
    Logica PLC simulata:

    FReleThtN (FeedBRele) - coil COIL_BASE[N]+3:
      OnThtN  (coil +4) scritto 1 -> dopo FRELE_DELAY s -> FReleThtN = 1
      OffThtN (coil +5) scritto 1 -> dopo FRELE_DELAY s -> FReleThtN = 0

    ReleThtN (StatoRele) - coil COIL_BASE[N]+2:
      Ricalcolato ogni volta che temperatura o setpoint cambiano:
        ReleThtN = 1  se  Setpoint(holding 101+N) > Temperatura(holding ADDR_T[N])
        ReleThtN = 0  altrimenti
    """

    def _rele_addr(self, n):  return COIL_BASE[n] + 2
    def _frele_addr(self, n): return COIL_BASE[n] + 3
    def _on_addr(self, n):    return COIL_BASE[n] + 4
    def _off_addr(self, n):   return COIL_BASE[n] + 5

    def _update_stato_rele(self, n):
        """Ricalcola ReleThtN in base a Setpoint vs Temperatura."""
        temp  = self.getValues(3, ADDR_T[n], count=1)[0]   # FC3 = holding
        setp  = self.getValues(3, ADDR_S[n], count=1)[0]
        stato = 1 if setp > temp else 0
        super().setValues(1, self._rele_addr(n), [stato])   # FC1 = coil
        log.info(
            f"  [PLC] Tht{n+1} StatoRele({self._rele_addr(n)}) = {stato} "
            f"(SP={setp} {'>' if stato else '<='} T={temp})"
        )

    def setValues(self, fc_as_hex, address, values):
        super().setValues(fc_as_hex, address, values)

        # ── Scrittura COIL: gestione OnTht / OffTht ──────────
        if fc_as_hex in (1, 5, 15):
            for i, val in enumerate(values):
                addr = address + i
                if not val:
                    continue
                n = next(
                    (n for n in range(13)
                     if addr in (self._on_addr(n), self._off_addr(n))),
                    None
                )
                if n is None:
                    continue

                is_on     = (addr == self._on_addr(n))
                new_state = 1 if is_on else 0
                action    = "ON" if is_on else "OFF"
                frele     = self._frele_addr(n)

                log.info(
                    f"  [PLC] Tht{n+1} {action} ricevuto "
                    f"-> FRele({frele}) = {new_state} tra {FRELE_DELAY}s"
                )

                def _delayed(ctx=self, fa=frele, st=new_state, zone=n+1, act=action):
                    time.sleep(FRELE_DELAY)
                    # setValues diretto sul parent per evitare ricorsione
                    ModbusDeviceContext.setValues(ctx, 1, fa, [st])
                    log.info(f"  [PLC] Tht{zone} {act} -> FRele({fa}) = {st}  [applicato]")

                threading.Thread(target=_delayed, daemon=True).start()

        # ── Scrittura HOLDING: ricalcola StatoRele se T o SP cambiano ──
        elif fc_as_hex in (3, 6, 16):
            for i in range(len(values)):
                addr = address + i
                # Temperatura cambiata?
                if addr in ADDR_T:
                    n = ADDR_T.index(addr)
                    self._update_stato_rele(n)
                # Setpoint cambiato?
                elif addr in ADDR_S:
                    n = ADDR_S.index(addr)
                    self._update_stato_rele(n)


# ============================================================
#  Costruzione slave TERMO BRIDGE
# ============================================================
def build_termo_slave() -> TermoDeviceContext:
    coil_vals = {}

    # poller01 - pompe (32..38)
    for addr in [32, 34, 35, 36, 37, 38]:
        coil_vals[addr] = 1

    # poller02 - allarme generale (73)
    coil_vals[73] = 0

    # poller03 - coil termostati 1-13
    for n in range(13):
        b = COIL_BASE[n]
        # StatoRele iniziale: 1 se setpoint > temperatura
        stato_rele = 1 if d10(SETP_C[n]) > d10(TEMP_C[n]) else 0
        coil_vals[b + 0] = 1           # TestTht
        coil_vals[b + 1] = 0           # AllTht
        coil_vals[b + 2] = stato_rele  # ReleTht  (StatoRele)
        coil_vals[b + 3] = FRELE[n]    # FReleTht (FeedBRele)
        coil_vals[b + 4] = FRELE[n]    # OnTht
        coil_vals[b + 5] = 0           # OffTht

    hold_vals = {}

    # poller04 - temperatura e umidita in decimi
    for n in range(13):
        hold_vals[ADDR_T[n]] = d10(TEMP_C[n])
        hold_vals[ADDR_U[n]] = d10(UMID_PC[n])

    # poller08 - setpoint in decimi (101..113)
    for n in range(13):
        hold_vals[ADDR_S[n]] = d10(SETP_C[n])

    # poller09/10/11 - fasce zona
    hold_vals[5005] = 1
    hold_vals[5020] = 1
    hold_vals[5035] = 1

    return TermoDeviceContext(
        di=make_block({}),
        co=make_block(coil_vals),
        hr=make_block(hold_vals),
        ir=make_block({}),
    )


# ============================================================
#  Costruzione slave PDC BRIDGE
# ============================================================
def build_pdc_slave() -> ModbusDeviceContext:
    hold_vals = {}

    hold_vals[1] = d10(42)    # TAccImp
    hold_vals[2] = d10(48)    # TContrACS
    hold_vals[3] = d10(38)    # TMandGEO
    hold_vals[4] = d10(32)    # TRitGEO
    hold_vals[5] = d10(40)    # TUscitaScambCI
    hold_vals[6] = d10(35)    # TIngressoScambCP

    hold_vals[113] = d10(22)  # SetTinStCC
    hold_vals[114] = d10(22)  # SetTinStCHP
    hold_vals[115] = d10(55)  # SetTACStCPDC
    hold_vals[116] = d10(20)  # SetTinSaCC
    hold_vals[117] = d10(20)  # SetTinSaCHP
    hold_vals[118] = d10(50)  # SetTACSESCPDC

    hold_vals[5003] = 1       # StatoPDC: 1=accesa
    hold_vals[5004] = 2       # ModoPDC:  2=riscaldamento

    return ModbusDeviceContext(
        di=make_block({}),
        co=make_block({}),
        hr=make_block(hold_vals),
        ir=make_block({}),
    )


# ============================================================
#  Thread drift temperatura (+-1 decimo ogni 15s)
#  Aggiorna anche StatoRele dopo ogni variazione
# ============================================================
def drift_thread(termo_ctx: ModbusServerContext):
    while True:
        time.sleep(15)
        try:
            slave = termo_ctx[1]
            for n, addr in enumerate(ADDR_T):
                cur = slave.getValues(3, addr, count=1)[0]
                delta = random.choice([-1, 0, 0, 0, 1])
                new_val = max(d10(15), min(d10(30), cur + delta))
                if new_val != cur:
                    # usa setValues di TermoDeviceContext per ricalcolare StatoRele
                    slave.setValues(3, addr, [new_val])
        except Exception as e:
            log.warning(f"drift: {e}")


# ============================================================
#  MAIN
# ============================================================
async def run_servers():
    termo_ctx = ModbusServerContext(devices={1: build_termo_slave()}, single=False)
    pdc_ctx   = ModbusServerContext(devices={1: build_pdc_slave()},   single=False)

    log.info("=" * 62)
    log.info("  Simulatore Modbus TCP - openHAB termostati + PDC")
    log.info("=" * 62)
    log.info(f"  {'#':<4} {'Zona':<22} {'T':>5} {'SP':>5} {'FRele':>6} {'Rele':>5}")
    log.info("  " + "-" * 50)
    for n in range(13):
        stato = 1 if d10(SETP_C[n]) > d10(TEMP_C[n]) else 0
        log.info(
            f"  {n+1:<4} {ZONE_NOMI[n]:<22} "
            f"{d10(TEMP_C[n]):>5}  "
            f"{d10(SETP_C[n]):>5}  "
            f"{'ON' if FRELE[n] else 'OFF':>6}  "
            f"{'ON' if stato else 'OFF':>5}"
        )
    log.info("")
    log.info(f"  FReleTht delay = {FRELE_DELAY}s")
    log.info(f"  StatoRele = 1 se Setpoint > Temperatura")
    log.info("")

    # Verifica valori nel datastore
    slave = termo_ctx[1]
    log.info("=== VERIFICA VALORI INIZIALI NEL DATASTORE ===")
    for n in range(13):
        t = slave.getValues(3, ADDR_T[n], count=1)[0]
        s = slave.getValues(3, ADDR_S[n], count=1)[0]
        log.info(f"  Tht{n+1:<2} T={t} ({t/10:.1f}C)  SP={s} ({s/10:.1f}C)")
    log.info("")

    threading.Thread(target=drift_thread, args=(termo_ctx,), daemon=True).start()

    log.info("Avvio server TERMO su 192.168.188.54:5020 ...")
    log.info("Avvio server PDC   su 192.168.188.54:5021 ...")

    await asyncio.gather(
        StartAsyncTcpServer(context=termo_ctx, address=("192.168.188.54", 5020)),
        StartAsyncTcpServer(context=pdc_ctx,   address=("192.168.188.54", 5021)),
    )


if __name__ == "__main__":
    asyncio.run(run_servers())