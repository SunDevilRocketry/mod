# MOD

The mod library is a platform-independent utility library written in C that is used throughout SDR’s embedded C code. The library is composed of “components”, which are smaller libraries with dependencies on each other. Components may be further broken down into modules based on complexity.

Historically, the “mod” library was the sole common library at SDR used to house drivers and utilities side-by-side for use across projects. As of version 3.1.0, it has been made completely independent of external components except for any C standard library implementation.

The components in “mod” are optimized for embedded projects that run in low-power, low-memory, and low-speed environments. It is designed to be integrated into safety critical projects and follows SDR’s highest standard for software quality assurance. No dynamic memory allocation is used – all memory is scoped statically, on the stack, or provided by the integrating project.

## QUALITY ASSURANCE PRACTICES

The primary guiding standard for this project is the [SDR Software Quality Assurance Standard](https://docs.google.com/document/d/18T5YLIsYWkMpDS0G6ab43WqmqcFqEYc4my6OPZia8H4/edit?tab=t.0). This version of the library is intended to be used across all software projects, so it is certified as “Safety Critical”.

However, as a library, this project is unable to determine how it is used and is therefore unable to analyze its own hazards. Hazard analysis shall be performed by the integrating project. The architectural guidance for this project is therefore to let error handling be performed by whichever project is calling into it.

The mod library was designed so that no component sits on the system boundary. Validation is performed on “public” inputs to the library to prevent memory access faults and other memory related issues. Thus, the security surface of this software project is considered to be negligible.