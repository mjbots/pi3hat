# pi3hat C++ examples #

The examples that are compiled and are bundled with pi3hat binary distributions are the exact same examples as in the moteus repository here:  https://github.com/mjbots/moteus/tree/main/lib/cpp/examples

The only difference is that they are additionally linked with
[pi3hat_moteus_transport_register.cc](pi3hat_moteus_transport_register.cc)
which registers the pi3hat transport with the default transport
creation mechanism.  This will result in a pi3hat being used by default if it is available, command line options to select other transports, and command line options to configure the pi3hat.

If your software is intended to only operate with a pi3hat, you could
instead manually construct a `Pi3HatMoteusTransport`, or operate with
`Pi3Hat` interface directly.
