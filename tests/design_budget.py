#!/usr/bin/env python3
"""Reproducible timing/link budget and conservative 32-bit source RAM audit.

GCC emits 32-bit assembly only: no ARM executable or board CPU time is measured.
The C declarations come from the actual Keil project; libc/linker allowances
are added separately. Check the Keil map before treating this as linked SRAM.
"""
import argparse
import json
from pathlib import Path
import re
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
HEADERS = {
    "stdint.h": "typedef unsigned char uint8_t; typedef signed char int8_t; typedef unsigned short uint16_t; typedef short int16_t; typedef unsigned int uint32_t; typedef int int32_t; typedef unsigned long long uint64_t; typedef long long int64_t; typedef unsigned int uintptr_t; typedef int intptr_t;\n",
    "stddef.h": "typedef unsigned int size_t; typedef int ptrdiff_t;\n#define NULL ((void *)0)\n#define offsetof(t,m) __builtin_offsetof(t,m)\n",
    "string.h": "#include <stddef.h>\nextern void *memcpy(void*,const void*,size_t); extern void *memset(void*,int,size_t); extern int strcmp(const char*,const char*); extern int strncmp(const char*,const char*,size_t); extern size_t strlen(const char*);\n",
    "stdio.h": "#include <stddef.h>\nextern int snprintf(char*,size_t,const char*,...); extern int printf(const char*,...);\n",
    "stdlib.h": "#include <stddef.h>\nextern void *malloc(size_t); extern void free(void*);\n",
}


def source_ram():
    project = ROOT / "MDK-ARM/he_thong_nhung.uvprojx"
    tree = ET.parse(project)
    includes = tree.find(".//Cads/VariousControls/IncludePath").text.split(";")
    sources = [project.parent / node.text.replace("\\", "/")
               for node in tree.findall(".//Files/File/FilePath") if node.text.endswith(".c")]
    counts = {}
    with tempfile.TemporaryDirectory(prefix="stm32_ram_audit_") as tmp:
        directory = Path(tmp)
        for name, content in HEADERS.items(): (directory / name).write_text(content)
        flags = ["gcc", "-m32", "-S", "-Os", "-ffreestanding", "-nostdinc", "-I" + str(directory),
                 "-DUSE_HAL_DRIVER", "-DSTM32F103xB", *["-I" + str(project.parent / p) for p in includes]]
        for index, path in enumerate(sources):
            out = directory / f"source_{index}.s"
            subprocess.run([*flags, str(path), "-o", str(out)], check=True, capture_output=True)
            section, objects = "", {}
            for line in out.read_text().splitlines():
                common = re.match(r"\s*\.comm\s+([^,]+),(\d+)", line)
                if common:
                    objects[common[1]] = int(common[2]); continue
                if re.match(r"\s*\.(data|bss)\s*$", line): section = line.strip()
                elif re.match(r"\s*\.section\s+", line): section = line.strip().split()[1].split(",")[0]
                size = re.match(r"\s*\.size\s+([^,]+),\s*(\d+)\s*$", line)
                if size and section.startswith((".data", ".bss")): objects[size[1]] = int(size[2])
            counts[str(path.resolve().relative_to(ROOT))] = sum(objects.values())
    startup = (ROOT / "MDK-ARM/startup_stm32f103xb.s").read_text()
    stack = int(re.search(r"Stack_Size\s+EQU\s+(0x[0-9A-Fa-f]+)", startup)[1], 16)
    heap = int(re.search(r"Heap_Size\s+EQU\s+(0x[0-9A-Fa-f]+)", startup)[1], 16)
    estimate = sum(counts.values()) + stack + heap + 512 + 2048
    assert estimate < 20480, "Source estimate plus allowances exceeds STM32F103C8 SRAM"
    return {"source_objects_by_file_bytes": counts, "source_objects_bytes": sum(counts.values()),
            "stack_reserved_bytes": stack, "heap_reserved_bytes": heap,
            "alignment_allowance_bytes": 512, "libc_linker_allowance_bytes": 2048,
            "estimated_total_with_allowances_bytes": estimate, "chip_sram_bytes": 20480,
            "estimated_remaining_bytes": 20480 - estimate,
            "qualification": "32-bit GCC source-layout estimate; not an ARM/Keil linked memory map"}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()
    # 250 samples -> 12 packets of 62 bytes; each packet carries <=21 samples.
    rows = []
    for rate in (32000, 50000, 80000, 100000):
        rows.append({"rate_hz": rate, "timer_period_us": 1e6 / rate,
                     "adc_and_trigger_us": 7.1805555556, "dma_half_interval_ms": 250000 / rate,
                     "fifo_time_ms": 2048000 / rate, "protocol_bytes_per_s": rate / 250 * 12 * 62,
                     "packets_per_s": rate / 250 * 12, "cpu_cycles_per_21_samples": 72000000 * 21 / rate,
                     "cpu_cycles_per_packet_average": 72000000 * 250 / (12 * rate),
                     "default_capture_s": 100000 / rate, "million_sample_capture_s": 1000000 / rate})
    report = {"app": "STM32 ADC Data Logger", "design_max_rate_hz": 100000,
              "default_rate_hz": 50000, "default_samples": 100000, "max_samples": 1000000,
              "sample_limit_reason": "Finite-run PC memory/CSV/time policy, not MCU full-capture RAM",
              "adc_trigger_period_bound_hz": 1e6 / 7.1805555556,
              "raw_adc_bytes_at_max_samples": 2000000,
              "binary_bytes_at_max_samples_excluding_control": 1000000 // 250 * 12 * 62,
              "rows": rows, "ram_audit": source_ram(),
              "uart": {"baud": 115200, "frame_bits_per_byte": 10,
                       "wire_payload_bytes_per_s": 11520, "protocol_bytes_per_sample_budget": 744 / 250,
                       "ideal_sample_rate_before_margin_hz": 11520 / (744 / 250),
                       "configured_max_rate_hz": 3000, "default_rate_hz": 1000,
                       "default_samples": 10000, "max_rate_protocol_bytes_per_s": 3000 * 744 / 250,
                       "wire_headroom_fraction": 1 - (3000 * 744 / 250) / 11520},
              "cpu_time_status": "Measured on board using DWT max copy/packet cycles in MTR; no host timing claim"}
    if args.out: args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__": main()
