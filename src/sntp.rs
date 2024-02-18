use core::mem;

use embassy_net::{
    udp::{RecvError, UdpSocket},
    IpEndpoint, Ipv4Address,
};

const SNTP_TIME_OFFSET: u32 = 2_208_988_800;

const SNTP_PACKET_SIZE: usize = 48;

#[inline]
fn read_be_u32(input: &[u8]) -> u32 {
    let (int_bytes, _) = input.split_at(mem::size_of::<u32>());
    u32::from_be_bytes(int_bytes.try_into().unwrap())
}

/// Default public NTP address.
pub const NTP_ADDR: Ipv4Address = Ipv4Address([192, 168, 178, 30]);
pub const NTP_PORT: u16 = 123;

const NANOSECONDS_PER_SECOND: u64 = 1_000_000_000;
const FRACTIONAL_PART_MAX: u64 = u32::MAX as u64;

#[derive(Debug)]
pub struct UnixTimeStamp {
    pub secs: i64,
    pub nanos: u32,
}

pub async fn get_timestamp_ntp(socket: &mut UdpSocket<'_>) -> UnixTimeStamp {
    let mut packet = [0u8; SNTP_PACKET_SIZE];
    packet[0] = (3 << 6) | (4 << 3) | 3;
    socket
        .send_to(
            &packet,
            IpEndpoint {
                addr: embassy_net::IpAddress::Ipv4(NTP_ADDR),
                port: NTP_PORT,
            },
        )
        .await
        .unwrap();
    let raw_time = recv_packet(socket, &mut packet).await.unwrap();
    let raw_secs = raw_time.secs;
    let nanos = (((raw_time.frac as u64) * NANOSECONDS_PER_SECOND) / FRACTIONAL_PART_MAX) as u32;
    UnixTimeStamp {
        secs: (raw_secs - SNTP_TIME_OFFSET) as i64,
        nanos,
    }
}

#[inline]
async fn recv_packet(
    socket: &mut UdpSocket<'_>,
    packet: &mut [u8],
) -> Result<SntpTimestamp, &'static str> {
    match socket.recv_from(packet).await {
        Ok((recv, _)) => {
            if recv != SNTP_PACKET_SIZE {
                return Err("Invalid SNTP packet size received");
            }
            let hdr = packet[0];
            let vn = (hdr & 0x38) >> 3;
            if vn != 4 {
                return Err("Server returned wrong SNTP version {vn}, expected 4.");
            }
            let mode = hdr & 0x7;
            if mode != 4 && mode != 5 {
                return Err("Not a SNTP server reply");
            }
            let timestamp = SntpTimestamp {
                secs: read_be_u32(&packet[40..44]),
                frac: read_be_u32(&packet[44..48]),
            };
            Ok(timestamp)
        }
        Err(RecvError::Truncated) => Err("Invalid SNTP packet size received"),
    }
}

/// SNTP timestamp.
#[derive(Debug)]
pub struct SntpTimestamp {
    /// Seconds since era epoch.
    pub secs: u32,
    /// Fraction of second.
    pub frac: u32,
}
