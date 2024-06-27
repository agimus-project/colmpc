{
  description = "Description for the project";

  inputs = {
    nixpkgs.url = "github:nim65s/nixpkgs/hpp-fcl-3-pre";
    nur = {
      url = "github:nim65s/nur-packages/hpp-fcl-3-pre";
      inputs = {
        flake-parts.follows = "flake-parts";
        nixpkgs.follows = "nixpkgs";
        treefmt-nix.follows = "treefmt-nix";
      };
    };
    flake-parts = {
      url = "github:hercules-ci/flake-parts";
      inputs.nixpkgs-lib.follows = "nixpkgs";
    };
    treefmt-nix = {
      url = "github:numtide/treefmt-nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    inputs@{ flake-parts, ... }:
    flake-parts.lib.mkFlake { inherit inputs; } {
      imports = [ inputs.treefmt-nix.flakeModule ];
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "aarch64-darwin"
        "x86_64-darwin"
      ];
      perSystem =
        {
          config,
          pkgs,
          self',
          system,
          ...
        }:
        {
          _module.args.pkgs = import inputs.nixpkgs {
            inherit system;
            overlays = [ inputs.nur.overlays.default ];
          };
          devShells.default = pkgs.mkShell {
            nativeBuildInputs = [ config.treefmt.build.wrapper ];
            inputsFrom = [ self'.packages.default ];
            packages = [ (pkgs.python3.withPackages (p: [ p.numpy ])) ];
          };
          packages = {
            py-colmpc = pkgs.python3Packages.callPackage ./package.nix { pythonSupport = true; };
            default = self'.packages.py-colmpc;
          };
          treefmt = {
            projectRootFile = "flake.nix";
            programs = {
              deadnix.enable = true;
              nixfmt-rfc-style.enable = true;
              #ruff = {
              #check = true;
              #format = true;
              #};
            };
          };
        };
    };
}
