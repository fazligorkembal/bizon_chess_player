from joint_bench.protocol import Event, Log, Reply, Sample, Unknown, parse_line


def test_parses_a_sample_line():
    assert parse_line("T,42,123456789012,-1,2,-16384,0,32767,-32768\r\n") == Sample(
        42, 123456789012, (-1, 2, -16384), (0, 32767, -32768))


def test_a_truncated_or_garbled_sample_is_unknown_not_an_exception():
    assert isinstance(parse_line("T,42,1234,-1,2"), Unknown)
    assert isinstance(parse_line("T,42,12x4,-1,2,3,4,5,6"), Unknown)


def test_parses_events_replies_and_logs():
    assert parse_line("E,7,mark:tap-1\n") == Event(7, "mark:tap-1")
    assert parse_line("OK stream on") == Reply(True, "stream on")
    assert parse_line("ERR unknown command") == Reply(False, "unknown command")
    assert parse_line("# imu: reset") == Log("imu: reset")
    assert isinstance(parse_line("\x00\x7fgarbage"), Unknown)


def test_reply_fields_are_key_value_pairs():
    reply = parse_line("OK imu chip=MPU6050 who=0x68 rate=1000 lsb_per_g=8192")
    assert reply.fields() == {"chip": "MPU6050", "who": "0x68", "rate": "1000",
                              "lsb_per_g": "8192"}
