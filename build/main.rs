use std::env;
use std::fs::{File};
use std::path::{Path};
use std::io::Write;
use csv::{ReaderBuilder, Trim};

//use serde::{Serialize, Deserialize};
use rstar::{RTree};

pub mod geomag_record;


use geomag_record::GeomagRecord;



impl GeomagRecord {
    pub fn new_from_csv(input: &csv::StringRecord) -> Self {
        //csv consists of: lat_deg, lon_deg, mag_x, mag_y, mag_z
        GeomagRecord {
            lat_deg: input.get(0).unwrap().parse().unwrap(),
            lon_deg: input.get(1).unwrap().parse().unwrap(),
            mag_x: input.get(2).unwrap().parse().unwrap(),
            mag_y: input.get(3).unwrap().parse().unwrap(),
            mag_z: input.get(4).unwrap().parse().unwrap(),
        }
    }

}

fn do_parse_geomag(fin: &mut File, fout: &mut File) {

    let mut rdr = ReaderBuilder::new()
        .comment(Some(b'#'))
        .trim(Trim::All)
        .from_reader(fin);

    let mut rec_list = vec!();
    while let Some(result) = rdr.records().next() {
        let record = result.unwrap();
        let geo_rec = GeomagRecord::new_from_csv(&record);
        rec_list.push(geo_rec);
    }
    println!("rec list len: {}",  rec_list.len());

    let rtree = RTree::bulk_load(rec_list);
    println!("rtree size: {}",  rtree.size());


    let serialized = serde_json::to_string_pretty(&rtree).unwrap();
    //println!("serialized = {}", serialized);

    fout.write_all(serialized.as_bytes()).unwrap();
}

pub fn main() {
    let out_dir = env::var("OUT_DIR").unwrap();
    let dest_path = Path::new(&out_dir).join("earth_mag.json");
    println!("dest_path: {:?}", dest_path);
    let mut fout = File::create(&dest_path).unwrap();

    let src_dir = env::current_dir().unwrap();
    let data_file_path = src_dir.join("./data/geomag.csv");
    println!("data_file_path: {:?}", data_file_path);
    let mut fin: File = File::open(data_file_path).unwrap();

    do_parse_geomag(&mut fin, &mut fout);

    //TODO verify this works if we add new files OR updated files
    println!("cargo:rerun-if-changed=data/");
}


#[cfg(test)]
mod tests {
    #[test]
    fn test_mag_from_declination() {
        assert_eq!(0,1);
    }
}