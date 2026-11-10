import csv
import struct
import time
from pathlib import Path
from elftools.elf.elffile import ELFFile
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target

root = Path(__file__).parent
with (root / 'objects/imu_readout.axf').open('rb') as f:
    symbols = {s.name: s['st_value'] for s in ELFFile(f).get_section_by_name('.symtab').iter_symbols()}

with ConnectHelper.session_with_chosen_probe(
    unique_id='809420A0-FC012CC50F38', target_override='at32f405kbu7-4',
    pack='D:/DevEnv/AT32/ArteryTek.AT32F402_405_DFP.2.1.6.pack',
    frequency=1000000, connect_mode='attach') as session:
    target = session.target
    target.reset(reset_type=Target.ResetType.SYSRESETREQ)
    time.sleep(1)
    chip_id = target.read32(symbols['imu_id'])
    error = target.read32(symbols['imu_error'])
    print('WHO_AM_I:', hex(chip_id), 'imu_error:', error)
    if chip_id != 0x70 or error:
        raise SystemExit('IMU initialization failed; no valid six-axis samples.')
    rows = []
    for _ in range(100):
        target.halt()
        try:
            row = [target.read32(symbols[n]) for n in ['heartbeat_ms', 'imu_id', 'imu_error', 'sample_count']]
            for n in ['accel_g', 'gyro_dps']:
                row.extend(struct.unpack('<3f', bytes(target.read_memory_block8(symbols[n], 12))))
        finally:
            target.resume()
        rows.append(row)
        time.sleep(0.05)
    with (root / 'capture.csv').open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['ms', 'id', 'error', 'sample_count', 'ax_g', 'ay_g', 'az_g', 'gx_dps', 'gy_dps', 'gz_dps'])
        writer.writerows(rows)
    print('First:', rows[0])
    print('Last: ', rows[-1])
    print('Config:', target.read_memory_block8(symbols['imu_config'], 8))
    for i, name in enumerate(['ax_g', 'ay_g', 'az_g', 'gx_dps', 'gy_dps', 'gz_dps'], 4):
        values = [r[i] for r in rows]
        print(name, 'min/mean/max:', min(values), sum(values)/len(values), max(values))
