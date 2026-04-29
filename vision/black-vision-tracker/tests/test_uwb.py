from black_vision.uwb import parse_anchor_rcv_line


def test_parse_anchor_rcv_line_supports_distance_output() -> None:
    measurement = parse_anchor_rcv_line(
        "+ANCHOR_RCV=DRONE001,4,PING,183 cm,-78 dBm",
        anchor_id="arm_anchor",
        anchor_address="REYAX001",
        timestamp_s=12.0,
    )

    assert measurement is not None
    assert measurement.anchor_id == "arm_anchor"
    assert measurement.tag_address == "DRONE001"
    assert measurement.payload == "PING"
    assert measurement.distance_m == 1.83
    assert measurement.rssi == -78.0
    assert measurement.timestamp_s == 12.0
