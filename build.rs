fn main() {
	println!("cargo:rustc-link-search=native=lib");
	println!("cargo:rustc-link-search=native=/usr/lib");
	println!("cargo:rustc-link-lib=static=BulletDynamics_gmake_x64_debug");
	println!("cargo:rustc-link-lib=static=BulletCollision_gmake_x64_debug");
	println!("cargo:rustc-link-lib=static=LinearMath_gmake_x64_debug");
	println!("cargo:rustc-link-lib=stdc++");
}