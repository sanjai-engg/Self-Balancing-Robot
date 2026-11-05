# Testing and Results

## Quantitative results

This repository does not currently publish an independently verified, repeatable test dataset. No numerical performance table is included because the test conditions, measurement logs, and repeatability information are not available in the current project record.

## Historical source-code note

The firmware sketch contains a comment reporting a 36.4-second balancing run with a 3.1° angle standard deviation and no motor saturation at gains (K_p=10.0), (K_i=0.0), and (K_d=0.8).

That statement is retained only as a historical note from the source code. The comment does not provide a complete test protocol or raw data, so it should not be presented as an independently verified benchmark.

## Control behaviour and limitations

The supplied firmware regulates body angle only. It does not read wheel encoders or implement wheel-speed, position, or steering control. The robot may therefore drift while maintaining an approximately upright body angle.

The source code and dashboard expose sensor telemetry, controller terms, loop-frequency information, and step-response measurements. These features can support future testing, but their availability does not itself constitute measured performance results.
