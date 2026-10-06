# Changelog

All notable changes to this library are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning](https://semver.org/).

## [Unreleased]

## [0.1.0] - TBD

### Added
- `Tuf2000FlowMeter`: non-blocking eModbus RTU driver for the TUF-2000 (flow rate, positive
  totalizer, signal quality/strength, fluid sound speed).
- `Config` struct for pins (incl. optional RS-485 DE/RE pin), baud rate, slave ID, register map,
  poll interval, stuck-request timeout, calibration multiplier and totalizer jump limit.
- Optional `setLogger()` callback.
- `Tuf2000Protocol`: host-testable low-word-first REAL4 decoding, reply-length validation,
  totalizer jump guard and poll tracker with stale-reply detection.
- Native Unity tests and a basic example.
