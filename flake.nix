{
  description = "Tang Nano 20K SPI Flash Emulator - Open source FPGA toolchain";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = with pkgs; [
            # Synthesis
            yosys

            # Place and route for Gowin FPGAs
            nextpnr
            
            # Gowin-specific tools (apicula/apycula)
            # Provides gowin_pack, gowin_unpack, and nextpnr-gowin support
            python3Packages.apycula

            # Programming tool
            openfpgaloader

            # Useful utilities
            gtkwave        # Waveform viewer
            verilator      # Verilog simulator/linter
            
            # Build tools
            gnumake

            # Rust toolchain for spi-flash-tool
            cargo
            rustc
            pkg-config
            udev
          ];

          shellHook = ''
            echo "Tang Nano 20K SPI Flash Emulator Development Environment"
            echo ""
            echo "Available tools:"
            echo "  yosys            - Verilog synthesis"
            echo "  nextpnr-gowin    - Place and route for Gowin FPGAs"
            echo "  gowin_pack       - Bitstream packing (from apycula)"
            echo "  openFPGALoader   - FPGA programming"
            echo "  verilator        - Verilog linting/simulation"
            echo "  gtkwave          - Waveform viewer"
            echo "  cargo            - Rust build tool"
            echo ""
            echo "Build commands:"
            echo "  make             - Build bitstream"
            echo "  make prog        - Program FPGA (volatile)"
            echo "  make flash       - Program to flash (persistent)"
            echo "  make clean       - Clean build artifacts"
            echo "  make tool        - Build spi-flash-tool"
            echo ""
          '';
        };

        # Package for building the bitstream
        packages.default = pkgs.stdenv.mkDerivation {
          pname = "tang-nano-20k-spi-flash";
          version = "0.1.0";

          src = ./.;

          nativeBuildInputs = with pkgs; [
            yosys
            nextpnr
            python3Packages.apycula
            gnumake
          ];

          buildPhase = ''
            make
          '';

          installPhase = ''
            mkdir -p $out
            cp build/*.fs $out/ || true
            cp build/*.json $out/ || true
          '';
        };
      }
    );
}
