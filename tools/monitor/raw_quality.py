from __future__ import annotations

from dataclasses import dataclass, field

from protocol import StatusPacket


UINT16_MODULO = 0x10000


@dataclass
class DiagnosticSnapshot:
    """PC-side interpretation of the latest MCU STATUS counters."""

    pc_missing_after_tx_done: int = 0
    tx_inflight: int = 0
    pc_received_raw: int = 0
    pc_expected_raw: int = 0
    pc_missing_raw: int = 0


@dataclass
class RawQualityStats:
    """Track device-sequence gaps for the fixed-rate Raw packet stream."""

    received_count: int = 0
    expected_count: int = 0
    missing_count: int = 0
    last_sequence: int | None = None
    latest_status: StatusPacket | None = None
    latest_diagnostic: DiagnosticSnapshot = field(default_factory=DiagnosticSnapshot)

    def reset(self) -> None:
        self.received_count = 0
        self.expected_count = 0
        self.missing_count = 0
        self.last_sequence = None
        self.latest_status = None
        self.latest_diagnostic = DiagnosticSnapshot()

    def observe(self, sequence: int) -> int:
        sequence &= 0xFFFF
        self.received_count += 1

        if self.last_sequence is None:
            self.last_sequence = sequence
            self.expected_count = 1
            return 0

        delta = (sequence - self.last_sequence) % UINT16_MODULO
        self.last_sequence = sequence

        if delta == 0:
            return 0

        missing_before = delta - 1
        self.expected_count += delta
        self.missing_count += missing_before
        return missing_before

    def observe_status(self, status: StatusPacket) -> DiagnosticSnapshot:
        self.latest_status = status
        pc_missing_after_tx_done = max(status.tx_done_counter - self.received_count, 0)
        tx_inflight = max(status.tx_start_counter - status.tx_done_counter, 0)
        self.latest_diagnostic = DiagnosticSnapshot(
            pc_missing_after_tx_done=pc_missing_after_tx_done,
            tx_inflight=tx_inflight,
            pc_received_raw=self.received_count,
            pc_expected_raw=self.expected_count,
            pc_missing_raw=self.missing_count,
        )
        return self.latest_diagnostic

    @property
    def loss_rate(self) -> float:
        if self.expected_count == 0:
            return 0.0
        return self.missing_count / self.expected_count
