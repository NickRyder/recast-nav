// Altered Rust translation of layer data in DetourTileCacheBuilder.cpp.
// Copyright (c) 2009-2010 Mikko Mononen. Zlib: vendor/recastnavigation/License.txt.
//! Native TileCache v1 layer bytes. Compression is caller supplied; the header
//! does not identify a codec or store the cell sizes. Header fields use host byte
//! order. Endian conversion leaves the codec's payload untouched.
use crate::{layers::HeightfieldLayer, nav::*};

pub const MAGIC: u32 = 0x4454_4c52;
pub const VERSION: u32 = 1;
pub const HEADER_SIZE: usize = 56;

#[derive(Clone, Debug, PartialEq)]
pub struct Header {
    pub magic: u32,
    pub version: u32,
    pub coordinate: [i32; 3],
    pub bounds: [Point; 2],
    pub height_range: [u16; 2],
    pub width: u8,
    pub height: u8,
    /// Lower/upper [X, Z] grid coordinates.
    pub usable_bounds: [[u8; 2]; 2],
    pub padding: [u8; 2],
}
impl Default for Header {
    fn default() -> Self {
        Self {
            magic: MAGIC,
            version: VERSION,
            coordinate: [0; 3],
            bounds: [[0.; 3]; 2],
            height_range: [0; 2],
            width: 0,
            height: 0,
            usable_bounds: [[0; 2]; 2],
            padding: [0; 2],
        }
    }
}
impl Header {
    pub fn to_bytes(&self) -> [u8; HEADER_SIZE] {
        let mut bytes = [0; HEADER_SIZE];
        let words = [
            self.magic,
            self.version,
            self.coordinate[0] as u32,
            self.coordinate[1] as u32,
            self.coordinate[2] as u32,
        ];
        for (i, word) in words
            .into_iter()
            .chain(self.bounds.into_iter().flatten().map(f32::to_bits))
            .enumerate()
        {
            bytes[i * 4..i * 4 + 4].copy_from_slice(&word.to_ne_bytes());
        }
        bytes[44..46].copy_from_slice(&self.height_range[0].to_ne_bytes());
        bytes[46..48].copy_from_slice(&self.height_range[1].to_ne_bytes());
        bytes[48..54].copy_from_slice(&[
            self.width,
            self.height,
            self.usable_bounds[0][0],
            self.usable_bounds[1][0],
            self.usable_bounds[0][1],
            self.usable_bounds[1][1],
        ]);
        bytes[54..56].copy_from_slice(&self.padding);
        bytes
    }
    pub fn parse(bytes: &[u8]) -> Result<Self, u32> {
        if bytes.len() < HEADER_SIZE {
            return Err(FAILURE | INVALID_PARAM);
        }
        let word = |p| u32::from_ne_bytes(bytes[p..p + 4].try_into().unwrap());
        if word(0) != MAGIC {
            return Err(FAILURE | WRONG_MAGIC);
        }
        if word(4) != VERSION {
            return Err(FAILURE | WRONG_VERSION);
        }
        Ok(Self {
            magic: word(0),
            version: word(4),
            coordinate: [word(8) as i32, word(12) as i32, word(16) as i32],
            bounds: std::array::from_fn(|side| {
                std::array::from_fn(|axis| f32::from_bits(word(20 + side * 12 + axis * 4)))
            }),
            height_range: [
                u16::from_ne_bytes(bytes[44..46].try_into().unwrap()),
                u16::from_ne_bytes(bytes[46..48].try_into().unwrap()),
            ],
            width: bytes[48],
            height: bytes[49],
            usable_bounds: [[bytes[50], bytes[52]], [bytes[51], bytes[53]]],
            padding: [bytes[54], bytes[55]],
        })
    }
    pub fn grid_size(&self) -> usize {
        usize::from(self.width) * usize::from(self.height)
    }
}
#[derive(Clone, Copy, Debug)]
pub struct CompressionResult {
    pub status: u32,
    pub size: usize,
}
/// The callbacks receive initialized output storage. As native, only the status
/// failure bit aborts a build/decompression; other status bits are discarded.
pub trait Compressor {
    fn max_compressed_size(&mut self, input_size: usize) -> usize;
    fn compress(&mut self, input: &[u8], output: &mut [u8]) -> CompressionResult;
    fn decompress(&mut self, input: &[u8], output: &mut [u8]) -> CompressionResult;
}
#[derive(Clone, Debug)]
pub struct Layer {
    pub header: Header,
    pub region_count: u8,
    grid_size: usize,
    grids: Vec<u8>,
}
impl Layer {
    /// Heights, areas, connections, regions. Offsets retain the allocation's
    /// original grid size, independently of later header edits, as native.
    pub fn grids(&self) -> [&[u8]; 4] {
        std::array::from_fn(|i| &self.grids[i * self.grid_size..(i + 1) * self.grid_size])
    }
    pub fn grids_mut(&mut self) -> [&mut [u8]; 4] {
        let (heights, rest) = self.grids.split_at_mut(self.grid_size);
        let (areas, rest) = rest.split_at_mut(self.grid_size);
        let (connections, regions) = rest.split_at_mut(self.grid_size);
        [heights, areas, connections, regions]
    }
    /// Cell sizes belong to cache parameters, not the compressed header.
    pub fn heightfield(&self, cell_size: f32, cell_height: f32) -> HeightfieldLayer {
        let [heights, areas, connections, _] = self.grids();
        Self::heightfield_from_grids(
            &self.header,
            cell_size,
            cell_height,
            [heights.to_vec(), areas.to_vec(), connections.to_vec()],
        )
    }
    /// Borrows the original four grid ranges without copying or reallocating.
    pub(crate) fn heightfield_mut(
        &mut self,
        cell_size: f32,
        cell_height: f32,
    ) -> (HeightfieldLayer<&mut [u8]>, &mut [u8]) {
        let header = self.header.clone();
        let [heights, areas, connections, regions] = self.grids_mut();
        (
            Self::heightfield_from_grids(
                &header,
                cell_size,
                cell_height,
                [heights, areas, connections],
            ),
            regions,
        )
    }
    fn heightfield_from_grids<T>(
        header: &Header,
        cell_size: f32,
        cell_height: f32,
        [heights, areas, connections]: [T; 3],
    ) -> HeightfieldLayer<T> {
        HeightfieldLayer {
            bounds: header.bounds,
            cell_size,
            cell_height,
            width: header.width.into(),
            height: header.height.into(),
            min_x: header.usable_bounds[0][0].into(),
            max_x: header.usable_bounds[1][0].into(),
            min_z: header.usable_bounds[0][1].into(),
            max_z: header.usable_bounds[1][1].into(),
            hmin: header.height_range[0],
            hmax: header.height_range[1],
            heights,
            areas,
            connections,
        }
    }
}
fn zeroed(size: usize) -> Result<Vec<u8>, u32> {
    let mut bytes = Vec::new();
    bytes
        .try_reserve_exact(size)
        .map_err(|_| FAILURE | OUT_OF_MEMORY)?;
    bytes.resize(size, 0);
    Ok(bytes)
}
pub fn build_layer<C: Compressor + ?Sized>(
    compressor: &mut C,
    header: &Header,
    heights: &[u8],
    areas: &[u8],
    connections: &[u8],
) -> Result<Vec<u8>, u32> {
    let size = header.grid_size();
    if [heights.len(), areas.len(), connections.len()]
        .iter()
        .any(|&n| n < size)
    {
        return Err(FAILURE | INVALID_PARAM);
    }
    let max = compressor.max_compressed_size(size * 3);
    if max > i32::MAX as usize - HEADER_SIZE {
        return Err(FAILURE | INVALID_PARAM);
    }
    let mut data = zeroed(HEADER_SIZE + max)?;
    data[..HEADER_SIZE].copy_from_slice(&header.to_bytes());
    let mut input = zeroed(size * 3)?;
    input[..size].copy_from_slice(&heights[..size]);
    input[size..size * 2].copy_from_slice(&areas[..size]);
    input[size * 2..].copy_from_slice(&connections[..size]);
    let result = compressor.compress(&input, &mut data[HEADER_SIZE..]);
    if result.status & FAILURE != 0 {
        return Err(result.status);
    }
    if result.size > max {
        return Err(FAILURE | INVALID_PARAM);
    }
    data.truncate(HEADER_SIZE + result.size);
    Ok(data)
}
pub fn decompress_layer<C: Compressor + ?Sized>(
    compressor: &mut C,
    data: &[u8],
) -> Result<Layer, u32> {
    let header = Header::parse(data)?;
    if data.len() > i32::MAX as usize {
        return Err(FAILURE | INVALID_PARAM);
    }
    let size = header.grid_size();
    let mut grids = zeroed(size * 4)?;
    let result = compressor.decompress(&data[HEADER_SIZE..], &mut grids);
    if result.status & FAILURE != 0 {
        return Err(result.status);
    }
    // Native ignores the reported decompressed size, including a value greater
    // than capacity when the callback's actual writes remain within its buffer.
    Ok(Layer {
        header,
        region_count: 0,
        grid_size: size,
        grids,
    })
}
pub fn swap_header_endian(data: &mut [u8]) -> bool {
    if data.len() < HEADER_SIZE {
        return false;
    }
    let magic = u32::from_ne_bytes(data[..4].try_into().unwrap());
    let version = u32::from_ne_bytes(data[4..8].try_into().unwrap());
    if (magic != MAGIC || version != VERSION)
        && (magic != MAGIC.swap_bytes() || version != VERSION.swap_bytes())
    {
        return false;
    }
    for p in (0..44).step_by(4) {
        data[p..p + 4].reverse();
    }
    data[44..46].reverse();
    data[46..48].reverse();
    true
}
