fn main() {
    #[cfg(feature = "k230")]
    build_k230();
}

#[cfg(feature = "k230")]
fn build_k230() {
    use std::env;
    use std::path::PathBuf;

    let target = env::var("TARGET").unwrap_or_default();
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let cpp_dir = manifest_dir.join("cpp");
    let sdk_dir = cpp_dir.join("k230_sdk");

    // Determine compiler based on target
    let compiler = if target.contains("riscv64") {
        "riscv64-linux-gnu-g++"
    } else {
        // Host compilation (for testing/development)
        "g++"
    };

    // Build the C++ shim
    let mut build = cc::Build::new();

    build
        .cpp(true)
        .file(cpp_dir.join("k230_capture.cpp"))
        .include(&sdk_dir)
        .flag("-std=c++17")
        .flag("-fPIC")
        .warnings(false); // SDK headers may have warnings

    // Only set compiler for cross-compilation
    if target.contains("riscv64") {
        build.compiler(compiler);
    }

    build.compile("k230_capture");

    // Link instructions for the final binary
    println!("cargo:rerun-if-changed=cpp/k230_capture.h");
    println!("cargo:rerun-if-changed=cpp/k230_capture.cpp");
    println!("cargo:rerun-if-changed=cpp/k230_sdk/");

    // When cross-compiling for K230, we need to link against the SDK libraries.
    // These are found on the board at /usr/lib/.
    //
    // For now, we create stub implementations in the C++ file that will be
    // replaced when linking against the real SDK libs on the target.
    //
    // On the K230 board, the required libraries are:
    //   - libmpi_vb.a (or .so)
    //   - libmpi_vicap.a (or .so)
    //   - libmpi_sys.a (or .so)
    //
    // To link against real SDK libs, set K230_SDK_LIB_PATH:
    //   K230_SDK_LIB_PATH=/path/to/k230/libs cargo build --features k230
    if let Ok(lib_path) = env::var("K230_SDK_LIB_PATH") {
        println!("cargo:rustc-link-search=native={}", lib_path);
        println!("cargo:rustc-link-lib=static=mpi_vb");
        println!("cargo:rustc-link-lib=static=mpi_vicap");
        println!("cargo:rustc-link-lib=static=mpi_sys");
    }
}
