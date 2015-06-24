fn main() {
	println!("cargo:rustc-link-search=native=lib");
	println!("cargo:rustc-link-search=native=/usr/lib");
	println!("cargo:rustc-link-lib=static=BulletDynamics_xcode4_x64_release");
	println!("cargo:rustc-link-lib=static=BulletCollision_xcode4_x64_release");
	println!("cargo:rustc-link-lib=static=LinearMath_xcode4_x64_release");
	println!("cargo:rustc-link-lib=stdc++");
}
