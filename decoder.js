// TTN Uplink Payload Formatter
// Matches device message format:
//   byte 0    : version   (uint8)
//   byte 1    : type      (uint8)  - 1 = band report
//   byte 2    : length    (uint8)  - total message length in bytes
//   byte 3-4  : band[0] (100 Hz)  dBFS x10, int16, little-endian
//   byte 5-6  : band[1] (400 Hz)  dBFS x10, int16, little-endian
//   byte 7-8  : band[2] (1 kHz)   dBFS x10, int16, little-endian
//   byte 9-10 : band[3] (4 kHz)   dBFS x10, int16, little-endian

function readInt16LE(bytes, offset) {
    var value = bytes[offset] | (bytes[offset + 1] << 8);
    if (value & 0x8000) {
        value = value - 0x10000;
    }
    return value;
}

function decodeUplink(input) {
    var bytes = input.bytes;
    var warnings = [];
    var errors = [];

    var HEADER_LEN = 3;
    var BAND_LABELS = ["100Hz", "400Hz", "1kHz", "4kHz"];

    if (bytes.length < HEADER_LEN) {
        return {
            data: {},
            errors: ["payload too short (" + bytes.length + " bytes) to contain header"]
        };
    }

    var version = bytes[0];
    var type = bytes[1];
    var length = bytes[2];

    // The length byte is the device's own record of how many bytes it sent.
    // Comparing it to the actual received byte count catches truncation or
    // corruption in transit before we trust anything else in the message.
    if (length !== bytes.length) {
        errors.push(
            "length field (" + length + ") does not match actual payload length (" +
            bytes.length + ") -- message may be truncated or corrupted"
        );
        return {
            data: { version: version, type: type, length: length },
            errors: errors
        };
    }

    if (type !== 1) {
        warnings.push("unrecognized message type: " + type);
        return {
            data: { version: version, type: type, length: length },
            warnings: warnings
        };
    }

    var expectedLen = HEADER_LEN + BAND_LABELS.length * 2;
    if (bytes.length !== expectedLen) {
        errors.push(
            "type=1 (band report) expects " + expectedLen + " bytes, got " + bytes.length
        );
        return {
            data: { version: version, type: type, length: length },
            errors: errors
        };
    }

    var bands_dbfs = {};
    for (var i = 0; i < BAND_LABELS.length; i++) {
        var raw = readInt16LE(bytes, HEADER_LEN + i * 2);
        bands_dbfs[BAND_LABELS[i]] = raw / 10.0;
    }

    return {
        data: {
            version: version,
            type: type,
            length: length,
            bands_dbfs: bands_dbfs
        },
        warnings: warnings.length ? warnings : undefined
    };
}