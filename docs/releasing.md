# Publishing the source repository

The public repository and source archives are separate from crates.io releases.
The source repository is [NickRyder/recast-nav](https://github.com/NickRyder/recast-nav).
The source release is tagged `v0.1.0`; the crates are not published to crates.io.
Prepare the source tree before making the repository public:

1. Check provenance, formatting, dependencies, the README example, and the
   headless Bevy example. Run native differential tests under both arithmetic
   policies and reference widths; record the actual compiler and platform.
2. Review tracked files and reachable history for local paths, development-only
   records, private material, and unwanted attribution. Keep raw development
   evidence separately. A reviewed source snapshot can start a fresh history;
   preserve the development repository before doing so.
3. Export with `git archive`. Confirm the archive contains both crates, the
   pinned native reference, test drivers, documentation, lockfile, and all license
   notices. Build and test an extracted copy to find missing-file dependencies.
4. Choose the public repository destination. Set real repository metadata,
   review visibility and maintainer access, then publish the reviewed snapshot.
   Confirm hosted CI after the remote exists; configured jobs alone are not
   evidence that a platform passed.

Example source archive:

```sh
git archive --format=tar.gz --prefix=recast-nav/ HEAD > recast-nav.tar.gz
```

`SOURCE_REVISION` is substituted with the commit ID by `git archive`. Distribute
tracked source rather than an entire working folder containing caches, local
records, or Git metadata.

## Registry packages

Both packages currently have `publish = false`. A crates.io release needs a
separate packaging pass: include the applicable license texts, check README and
`include_str!` paths, provide feature-gated native test support correctly, and
build each package from its packaged contents in a clean consumer project.
Generation is part of the core package, with no separate backend to publish.

Verify the minimum supported Rust version independently of the pinned development
toolchain. Keep API, behavioral compatibility, binary format, and performance
claims scoped to their actual evidence.
