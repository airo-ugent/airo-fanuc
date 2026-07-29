#!/usr/bin/env bash
# eno2-rt-setup.sh — RT hygiene for the FANUC robot link (PLAN §5.5, R1 A2/E2).
#
# The Stream Motion RX arrives on eno2 (192.168.1.101/24, point-to-point to the
# R-30iB controller). For the 125 Hz PLL-clocked TX loop to servo its phase off
# fresh RX timestamps, the NIC must NOT batch RX interrupts, and the RX IRQ must
# land on the RT core (31) rather than wherever irqbalance drifts it.
#
# This script:
#   1. disables RX interrupt coalescing on eno2 (rx-usecs 0, adaptive-rx off);
#   2. pins every eno2 RX-queue IRQ to core 31 (smp_affinity_list);
#   3. bans irqbalance from moving those IRQs (persist via IRQBALANCE_ARGS ban list).
#
# It is idempotent and prints what it changed. Run as root (sudo) on olifant AFTER
# the slice drop-ins are in place. Re-run after a NIC/driver reload (IRQ numbers
# can change). This tunes the HOST/procedure; the P6 HIL soak re-verifies on wire.
#
# Install (optional, to run at boot):
#   sudo cp eno2-rt-setup.sh /usr/local/sbin/eno2-rt-setup.sh
#   sudo chmod +x /usr/local/sbin/eno2-rt-setup.sh
#   # then a oneshot unit (After=network-online.target) invoking it, or run by hand.
# Usage:  sudo ./eno2-rt-setup.sh [IFACE] [RT_CORE]     (defaults: eno2 31)

set -euo pipefail

IFACE="${1:-eno2}"
RT_CORE="${2:-31}"

if [[ "${EUID}" -ne 0 ]]; then
    echo "eno2-rt-setup: must run as root (sudo)." >&2
    exit 1
fi

if [[ ! -d "/sys/class/net/${IFACE}" ]]; then
    echo "eno2-rt-setup: interface ${IFACE} not found — is the robot link up?" >&2
    exit 1
fi

echo "== eno2-rt-setup: iface=${IFACE} rt_core=${RT_CORE} =="

# 1) Kill RX interrupt coalescing. Some NICs reject one flag; apply what sticks.
echo "-- ethtool: disabling RX coalescing"
ethtool -C "${IFACE}" rx-usecs 0 adaptive-rx off 2>/dev/null \
    || ethtool -C "${IFACE}" rx-usecs 0 2>/dev/null \
    || echo "   (warn) ${IFACE} did not accept the coalescing knobs — check 'ethtool -c ${IFACE}'"
ethtool -c "${IFACE}" 2>/dev/null | grep -E 'rx-usecs:|Adaptive RX' || true

# 2) Pin every eno2 RX-queue IRQ to the RT core.
echo "-- IRQ affinity: pinning ${IFACE} RX IRQ(s) to core ${RT_CORE}"
mapfile -t IRQS < <(grep -E "${IFACE}(-rx|-tx-rx|-TxRx|[^0-9a-z]|\$)" /proc/interrupts | awk -F: '{gsub(/ /,"",$1); print $1}')
if [[ "${#IRQS[@]}" -eq 0 ]]; then
    # Fallback: match any line mentioning the iface at all.
    mapfile -t IRQS < <(awk -v n="${IFACE}" '$0 ~ n {gsub(/ /,"",$1); sub(/:/,"",$1); print $1}' /proc/interrupts)
fi
if [[ "${#IRQS[@]}" -eq 0 ]]; then
    echo "   (warn) no ${IFACE} IRQ lines found in /proc/interrupts — nothing pinned"
else
    for irq in "${IRQS[@]}"; do
        if [[ -w "/proc/irq/${irq}/smp_affinity_list" ]]; then
            echo "${RT_CORE}" > "/proc/irq/${irq}/smp_affinity_list" \
                && echo "   IRQ ${irq} -> core ${RT_CORE}" \
                || echo "   (warn) could not set affinity for IRQ ${irq}"
        fi
    done
fi

# 3) Ban irqbalance from moving those IRQs. Prefer a persistent ban list; fall back
#    to stopping the daemon for the session if the config path is unavailable.
echo "-- irqbalance: banning ${IFACE} IRQ(s) from rebalancing"
if [[ "${#IRQS[@]}" -gt 0 ]] && [[ -d /etc/default ]]; then
    ban_args=""
    for irq in "${IRQS[@]}"; do
        ban_args+=" --banirq=${irq}"
    done
    # IRQBALANCE_ARGS is honoured by the packaged irqbalance unit on Ubuntu.
    if grep -q '^IRQBALANCE_ARGS=' /etc/default/irqbalance 2>/dev/null; then
        sed -i "s|^IRQBALANCE_ARGS=.*|IRQBALANCE_ARGS=\"${ban_args# }\"|" /etc/default/irqbalance
    else
        echo "IRQBALANCE_ARGS=\"${ban_args# }\"" >> /etc/default/irqbalance
    fi
    echo "   wrote IRQBALANCE_ARGS=\"${ban_args# }\" to /etc/default/irqbalance"
    systemctl restart irqbalance 2>/dev/null \
        || echo "   (warn) could not restart irqbalance — 'sudo systemctl stop irqbalance' to hold the pin this session"
else
    systemctl stop irqbalance 2>/dev/null \
        && echo "   stopped irqbalance for this session (no /etc/default/irqbalance to persist a ban)" \
        || echo "   (warn) irqbalance not running / not present"
fi

echo "== eno2-rt-setup: done. Verify: cat /proc/irq/<N>/smp_affinity_list =="
