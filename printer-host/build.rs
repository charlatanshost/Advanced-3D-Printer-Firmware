// Build script for compiling RT-Labs fieldbus libraries
use std::env;
use std::path::PathBuf;

fn main() {
    let _out_dir = env::var("OUT_DIR").unwrap();
    let _manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();

    // ethercrab pulls in the pcap crate on Windows, which asks the linker for wpcap.lib from the
    // Npcap SDK even though nothing here calls it: ethercat_probe.rs loads Npcap itself when run.
    // An empty library satisfies the linker, so the app builds without the SDK.
    #[cfg(all(windows, not(feature = "windows-ethercat")))]
    {
        let dir = PathBuf::from(&_out_dir).join("empty-libs");
        std::fs::create_dir_all(&dir).expect("create folder for empty libraries");
        for name in ["wpcap.lib", "Packet.lib"] {
            std::fs::write(dir.join(name), b"!<arch>
").expect("write empty library");
        }
        println!("cargo:rustc-link-search=native={}", dir.display());
    }

    // Windows EtherCAT (Npcap) library configuration
    #[cfg(all(windows, feature = "windows-ethercat"))]
    {
        use std::process::Command;

        // Path to our created import libraries
        let lib_dir = format!("{}/target/npcap-libs", _manifest_dir);
        let lib_path = PathBuf::from(&lib_dir);

        // Check if we need to create import libraries
        let wpcap_lib = lib_path.join("wpcap.lib");
        let packet_lib = lib_path.join("packet.lib");

        if !wpcap_lib.exists() || !packet_lib.exists() {
            println!("cargo:warning=Creating Npcap import libraries...");

            // Create target directory
            std::fs::create_dir_all(&lib_path).unwrap_or_else(|e| {
                panic!("Failed to create lib directory: {}", e);
            });

            // Create definition files and import libraries
            let npcap_dll_path = "C:\\Windows\\System32\\Npcap";

            // Generate wpcap definition file and import library
            let wpcap_dll = format!("{}\\wpcap.dll", npcap_dll_path);
            if std::path::Path::new(&wpcap_dll).exists() {
                let wpcap_def = lib_path.join("wpcap.def");
                Command::new("gendef")
                    .arg(&wpcap_dll)
                    .current_dir(&lib_path)
                    .output()
                    .expect("Failed to run gendef for wpcap.dll");

                Command::new("dlltool")
                    .args(&["-d", "wpcap.def", "-l", "wpcap.lib", "-D"])
                    .arg(&wpcap_dll)
                    .current_dir(&lib_path)
                    .output()
                    .expect("Failed to create wpcap.lib");
            }

            // Generate packet definition file and import library
            let packet_dll = format!("{}\\Packet.dll", npcap_dll_path);
            if std::path::Path::new(&packet_dll).exists() {
                let packet_def = lib_path.join("Packet.def");
                Command::new("gendef")
                    .arg(&packet_dll)
                    .current_dir(&lib_path)
                    .output()
                    .expect("Failed to run gendef for Packet.dll");

                Command::new("dlltool")
                    .args(&["-d", "Packet.def", "-l", "packet.lib", "-D"])
                    .arg(&packet_dll)
                    .current_dir(&lib_path)
                    .output()
                    .expect("Failed to create packet.lib");
            }
        }

        // Use our created import libraries
        if wpcap_lib.exists() && packet_lib.exists() {
            println!("cargo:rustc-link-search=native={}", lib_dir);
            println!("cargo:rustc-link-lib=wpcap");
            println!("cargo:rustc-link-lib=packet");
            println!("cargo:warning=Using Npcap import libraries from: {}", lib_dir);
        } else {
            println!("cargo:warning=Failed to create or find Npcap import libraries");
        }

        // Tell the system where to find the DLLs at runtime
        println!("cargo:rustc-env=PATH=C:\\Windows\\System32\\Npcap;{}", env::var("PATH").unwrap_or_default());
    }

    // Disable fieldbus library compilation for now to allow Rust compilation
    // This will be re-enabled once the fieldbus submodules are properly configured

    println!("cargo:rerun-if-changed=build.rs");
    
    // Temporarily comment out fieldbus library builds
    /*
    // Base path to fieldbus libraries
    let fieldbus_libs_path = PathBuf::from(&manifest_dir)
        .parent()
        .unwrap()
        .join("fieldbus-libs");
    
    // Build P-Net (PROFINET)
    if fieldbus_libs_path.join("p-net").exists() {
        println!("cargo:rerun-if-changed=../fieldbus-libs/p-net");
        build_pnet(&fieldbus_libs_path.join("p-net"), &out_dir);
    }

    // Build M-Bus (Modbus RTU/TCP)
    if fieldbus_libs_path.join("m-bus").exists() {
        println!("cargo:rerun-if-changed=../fieldbus-libs/m-bus");
        build_mbus(&fieldbus_libs_path.join("m-bus"), &out_dir);
    }

    // Build C-Open (CANopen)
    if fieldbus_libs_path.join("c-open").exists() {
        println!("cargo:rerun-if-changed=../fieldbus-libs/c-open");
        build_copen(&fieldbus_libs_path.join("c-open"), &out_dir);
    }

    // Build C-Link (CC-Link IE)
    if fieldbus_libs_path.join("c-link").exists() {
        println!("cargo:rerun-if-changed=../fieldbus-libs/c-link");
        build_clink(&fieldbus_libs_path.join("c-link"), &out_dir);
    }

    // Build I-Link (IO-Link)
    if fieldbus_libs_path.join("i-link").exists() {
        println!("cargo:rerun-if-changed=../fieldbus-libs/i-link");
        build_ilink(&fieldbus_libs_path.join("i-link"), &out_dir);
    }
    */
}

fn build_pnet(pnet_path: &PathBuf, _out_dir: &str) {
    let mut build = cc::Build::new();
    
    // Add P-Net source files
    let src_path = pnet_path.join("src");
    if src_path.exists() {
        build
            .include(&src_path)
            .include(pnet_path.join("include"))
            .files(glob::glob(&format!("{}/**/*.c", src_path.display()))
                .expect("Failed to read P-Net source files")
                .filter_map(Result::ok))
            .define("PNET_MAX_FILENAME_SIZE", "240")
            .define("PNET_MAX_STATION_NAME_SIZE", "240")
            .warnings(false);

        // Platform-specific settings
        #[cfg(target_os = "windows")]
        {
            build.define("_WIN32", None);
            println!("cargo:rustc-link-lib=ws2_32");
            println!("cargo:rustc-link-lib=iphlpapi");
        }

        #[cfg(target_os = "linux")]
        {
            build.define("__linux__", None);
            println!("cargo:rustc-link-lib=pthread");
        }

        build.compile("pnet");
        println!("cargo:rustc-link-lib=static=pnet");
    }
}

fn build_mbus(mbus_path: &PathBuf, _out_dir: &str) {
    let mut build = cc::Build::new();
    
    let src_path = mbus_path.join("src");
    if src_path.exists() {
        build
            .include(&src_path)
            .include(mbus_path.join("include"))
            .files(glob::glob(&format!("{}/**/*.c", src_path.display()))
                .expect("Failed to read M-Bus source files")
                .filter_map(Result::ok))
            .define("MBUS_MAX_FRAME_SIZE", "256")
            .warnings(false);

        // Platform-specific settings
        #[cfg(target_os = "windows")]
        {
            build.define("_WIN32", None);
        }

        #[cfg(target_os = "linux")]
        {
            build.define("__linux__", None);
            println!("cargo:rustc-link-lib=pthread");
        }

        build.compile("mbus");
        println!("cargo:rustc-link-lib=static=mbus");
    }
}

fn build_copen(copen_path: &PathBuf, _out_dir: &str) {
    let mut build = cc::Build::new();
    
    let src_path = copen_path.join("src");
    if src_path.exists() {
        build
            .include(&src_path)
            .include(copen_path.join("include"))
            .files(glob::glob(&format!("{}/**/*.c", src_path.display()))
                .expect("Failed to read C-Open source files")
                .filter_map(Result::ok))
            .define("CO_MAX_NODES", "127")
            .warnings(false);

        // Platform-specific settings
        #[cfg(target_os = "windows")]
        {
            build.define("_WIN32", None);
        }

        #[cfg(target_os = "linux")]
        {
            build.define("__linux__", None);
            println!("cargo:rustc-link-lib=pthread");
        }

        build.compile("copen");
        println!("cargo:rustc-link-lib=static=copen");
    }
}

fn build_clink(clink_path: &PathBuf, _out_dir: &str) {
    let mut build = cc::Build::new();
    
    let src_path = clink_path.join("src");
    if src_path.exists() {
        build
            .include(&src_path)
            .include(clink_path.join("include"))
            .files(glob::glob(&format!("{}/**/*.c", src_path.display()))
                .expect("Failed to read C-Link source files")
                .filter_map(Result::ok))
            .define("CL_MAX_STATIONS", "64")
            .warnings(false);

        // Platform-specific settings
        #[cfg(target_os = "windows")]
        {
            build.define("_WIN32", None);
            println!("cargo:rustc-link-lib=ws2_32");
        }

        #[cfg(target_os = "linux")]
        {
            build.define("__linux__", None);
            println!("cargo:rustc-link-lib=pthread");
        }

        build.compile("clink");
        println!("cargo:rustc-link-lib=static=clink");
    }
}

fn build_ilink(ilink_path: &PathBuf, _out_dir: &str) {
    let mut build = cc::Build::new();
    
    let src_path = ilink_path.join("src");
    if src_path.exists() {
        build
            .include(&src_path)
            .include(ilink_path.join("include"))
            .files(glob::glob(&format!("{}/**/*.c", src_path.display()))
                .expect("Failed to read I-Link source files")
                .filter_map(Result::ok))
            .define("IOL_MAX_PORTS", "8")
            .warnings(false);

        // Platform-specific settings
        #[cfg(target_os = "windows")]
        {
            build.define("_WIN32", None);
        }

        #[cfg(target_os = "linux")]
        {
            build.define("__linux__", None);
            println!("cargo:rustc-link-lib=pthread");
        }

        build.compile("ilink");
        println!("cargo:rustc-link-lib=static=ilink");
    }
}