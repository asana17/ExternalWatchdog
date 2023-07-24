# The Safety Island Manual V1.0

## Index
1. [Description](#description)
1. [Node Descriptions](#node-descriptions)
1. [Prerequisites](#Prerequisites)
1. [Preparation](#preparation)
1. [How to Run](#how-to-run)
1. [Simulation Examples](#simulation-examples)

## Description
<a id="description"></a>
The source code can be accessed from the GitHub link: [https://github.com/tier4/safety_island](https://github.com/tier4/safety_island).

### Current Implementation
Current safety island implementation includes following functions.
1. Monitor ROS2 diagnostics from safety mechanisims.
1. Judge appropriate MRM behavior.
1. Judge the fault ECU and select an appropriate ECU to operate MRM.
1. Operate emergency stop without Autoware.
1. Switch control interface of ECUs.

Only running safety island functionality with planning simulator is now available. Implentation for real vehicle is currently WIP.

### Helper Functions
In the repository above, simulation helper functions are also included.
1. `hazard_status` conversion to add `self_recoverable` variable, which stands for judging whether MRM operation on self ECU is possible.
1. Dummy `hazard_status` publisher, which stands for fault/error simulation.

Details of those functions are written inside the [Node Description section](#node-descriptions).

## Node Descriptions
<a id="node-descriptions"></a>

For more info, please read README in [repo](https://github.com/tier4/safety_island).

### supervisor / supervisor_psim
https://github.com/tier4/safety_island/tree/549fe05a041e81e707d1ba7db02930058854c5c4/src/supervisor_psim

(`psim` stands for `planning_simulator`)

This node has Voter and Switch functionality in one ROS2 node.

Voter: Judge MRM behavior and operating ECU from self and external monitoring result.

Switch: Change Control Interface to connect with vehicle / psim.

### emergency_stop_operator
https://github.com/tier4/safety_island/tree/549fe05a041e81e707d1ba7db02930058854c5c4/src/emergency_stop_operator

Operate MRM emergency stop without Autoware.

### diagnostic_monitor
https://github.com/tier4/safety_island/tree/549fe05a041e81e707d1ba7db02930058854c5c4/src/diagnostic_monitor

Judge system `hazard_status` from ROS2 diagnostics. This node publish safety island style `hazard_status`.

### hazard_status_converter
https://github.com/tier4/safety_island/tree/549fe05a041e81e707d1ba7db02930058854c5c4/src/hazard_status_converter

This node converts Autoware style `hazard_status` to safety island style `hazard_status` by adding `self_recoverable` flag. The `self_recoverable` variable can be dynamically set with the config yaml. If the topic name written in the config is included in LF/SPF `hazard_status`, `self_recoverable` flag is set to false.

### dummy_hazard_status_publisher
https://github.com/tier4/safety_island/tree/549fe05a041e81e707d1ba7db02930058854c5c4/src/dummy_hazard_status_publisher

This node can generate `hazard_status` without subscribing diagnostics. `hazard_status` parameters can be set from the config yaml. Also, the `hazard_status` parameters can be dynamically change from `ros2 param set` interface.

### Node Diagram and Data Flow

#### Real Vehicle
[Drawio link](https://viewer.diagrams.net/?tags=%7B%7D&highlight=0000ff&edit=_blank&layers=1&nav=1&title=psim_safety_island#R7V1bd5s4EP41eWwP4s5jmibd7jZtd9PdTfclhxhs02JwMU7i%2FvoVMdgwGhtMJIFN23NaIy7GM9%2FcR9KZdjF7epe48%2Bl17Pnhmap4T2fa2zNVJbqm0%2F%2BykdV6xCLWemCSBF5%2B0XbgJvjp54NKProMPH9RuTCN4zAN5tXBURxF%2FiitjLlJEj9WLxvHYfVb5%2B7EZwZuRm7Ijv4beOl0PWobynb8Nz%2BYTItvJkp%2BZuYWF%2BcDi6nrxY%2BlIe3yTLtI4jhdf5o9XfhhRryCLuv7rnac3bxY4kdpkxum6d23xF99%2BePnxdflqz%2FcYHz76pVqrB%2Fz4IbL%2FBfnb5uuChIk8TLy%2FOwp5Ex78zgNUv9m7o6ys4%2BU6XRsms7C%2FDT7VvmLPvhJ6j%2BVhvK3fOfHMz9NVvSS%2FKxakDCHjKrnx49bBpCCqtMS8c18zM15Ptk8eksW%2BiGnDE6lP0cfrxXLO79%2B72mfVqt36j%2F6xSuiH0IlRQqVHLVKJU1lqWSYCJUMHmTCwaRwBtM4CMOLOIyT53u1sZH9zcbjKC2Nr%2F%2FQ8UWaxN%2F90hnz%2BQ8fghPFqVJcN1mKb%2FRWmeKaLQyY%2FcOlDYSXpZFuY6hURNFIZWh07QYRHbm8%2BJuhFv2RaZUkVUxFceQDYOZDbhhMIno4oqTz6fibjGQBNSTn%2BYlZ4HnZ16A8qHKJh3oAWLVZPmBs0ERxQes9Uk2lIVR1YVBlzczN8n5gSDUJywbLkIhUi2GCP%2FOTiR%2BNVs8ciOf0v3juJ25KqXo69k0HbhdBzJtct8tmODGKZ%2BM4Sd17Su4h8cJyuuaF00AqzDDTSPeUEeYk%2BzQAxjiI0ZAcm7BO91ClRDWNrpnBhtM3S0r4h2CRUf7EDblWZYemIbKBxkfC2GEishFR6mZ3Lh6DdDTNHpFRb5zRhqtojMfqaISJgGfemwavoJRAS2FJVEi3P8LZJ%2Bv%2B9u5S%2BRq8sy6V6edvhadfIqPvTfyb%2FJDqpWk8iSM3vNyOAixur%2FkQU921Ju83P01XeX7QXaZxlfieu5huGLWTrot4mYz8fYDJnb%2FUpaYt3eep59TLftpePiV%2B6KbBQzWbiNH8%2BdbzJHFXpQvmMQXnovTkz9lAif2qUWW%2FoRllBh58A%2F2wfoct%2Fzc%2F5gVyyHpyCz8c05FZHAXCjNJu47PLXAkwSppMgcSpz%2Fpug6G%2B2bnjjCRFqcH3E6rdBsEB3eicA2yWdFAcwDIqkjmgDosDTlFW640NUBvkX%2F3IO88qpllEEbqLRTCqkrjqpPlPQXqbn8k%2Bf83GXxv50dun0mVvV8VBrSPIyY8rfl2tH1c4fLV%2BXLmqhpYv1mOHuXuMe2ZC78wBgFj%2F8vyuPX6eBXSAoYMHrSnDPOhgB5ToQN0rlSq0GH9SbVCPPSU0k16BFNqX1iBlgldTFEoVx0K%2FSCxKG%2FRWnBJKm4bEx45SA%2BarOKHUhKrUVva%2BGLy%2BeK%2BDf4hYKWBzcJykYIv8ryWZwKVAdn6oyDz2RBg69yu4oYmtznahUyP6KzY3ZQelu7LD7W3PR8118QugiSOuitXbVPvo%2Fff3VWL9%2B5s2V93Fz3N73SBz9Mg0rOqDiCHIl4BfZCs1ahfeYErQuoWJKsnJgz8NRs8VwdHMo%2F9O3JRzxaPbmLu%2Be8FBYEwsABN%2BHU8NmvMkqiqlqqqsl%2BsqXjbVaWpT%2BxWrc3MwdagCBUVBFql%2BjwUbxmuuN6UoLrVfUvNaobCrSA7RVb6i83zzZz8JKO2yMv7%2Beny9PNm%2F5EmOPMFKR40jwPjO%2BbQCsfLEJn5PtvhHNNiB0nnevZj%2BJCD6bafObFOrqjNF0evUGaafpLkHBYCPTZ2dTMitsWnMYYUSWPOt5FCCf9ajbQajhfZ5gSujsLKPzjwrfJ6eyL4GG1NluzLcZJ9t3OrWfDk2MF82ZX0L89Uek02j1UJt9gSS%2FLxrs%2FogGz5IMCQLCehyfk5H3i1UBp339ejCgvVDajv1rmhJgRhl9UFqVEd7LVFI%2F7EVijhWTaGegLMmeeXjtWqvFVGqtf36N4N3iAnEdWEdWFVZUV4oKy%2FA%2FFAaAKRBuQbJJpyxeeD1toxeLV1c%2FZ9DxZbYtclcvu5j0waCngmTQTgJkwGUr8CIBp2dxB%2BNLZL9LQLvnYir9zD6hSS2Cwlq08ZQgtleQWrZgn53XZ4fXK86fKdYobjuT4aIU2K4Ftf9Svmwvaxw0Z%2BmsFZ1aKUNMbDW8BfeCWtwva3y9R7wxB4Da3m5525WmYJRvomsoCQsykd5wD9w6bvJdPqtWqy2FhPm7hwiyGLa%2BAvvVC3geodzYzIK6wYW83jVyGbdtRWgcFmLaHswy12LNKgkHDG5gW1E1%2BuRSm52zvfn0I2iIJrQx10Ui2CcDgN0yABkKRipDEDW5Xmp2eSY92htO2sTHEj32T6A9sTGwl5HG87xadytYe5QvPzzGzjshHUdH1SS2RZcrErFRanBHaf89L5Y4thKMkzbLnTa2kLThBGDaGiqPYPmQchsrxEL01TfxfILd0JwJ67y1sYSK68dy64g77Vii2ti2Rf7HFsTy%2BlYaf5twVwb%2BhTd3A9metBZR3DjSbj9aggmBKBXb5shhujVYeOpaPQ2KKIdcSgJUieq0XkoedqpKpA7QVdxlUtvNlc1qOSJik2bkcsBNn3VM5fNUky5LluhdI%2BtcwT6bMxmKG2tntZwGRZejUyIFn6IM85yVQTdrBqMbQ%2BCFhlV2K7Db%2FMgNjuwKC2dDWh8MotmE2j%2BsNUasW2ceCyavbw%2Fvw%2Bj9NPvH8iX76Z27Sy9H%2BieQldT96ebeHeL1E2Xi9PlhglddGR5f6nMwFJmV7NkRv%2FdzHq4W895uFrPeWCbIU6GO3AmJEGqbFK5gy3wNhhRgRVmpE9FKjOwoDQXldKGGIMRFrg6nN01f7Ag9irfjuHuuZfrZFkBRIUo2JrYikxmYN0Yw2AGgeuEYTv4SOUFFusOhBew2XHTY9oZM9C2jYFwQwdteHbnooF2M7Q26uK3lIQzrDSEgpsQurqnpCgKcowgJNAPOC0Oog8k0%2B%2BXj1%2FKTSA7ccvNTaBO%2FlC4Add7VtlCiVxuoC79QLhBVL1%2Bkw%2B57ECd%2BqGwQwF%2BfffsQP36oebumDx3sfFcd6lV1tP%2F69NNNvIx9k5pOTOiAL%2BeWBL3ScVpz%2Fr176PNhlDvI6qtouPcjxPW0ntAazQCGEREu2klKRobOi%2FnYK2oTXghPvQigFYa4l2iwBUWeqG7PPWTWBaW0JVLLDRO7QWxVLCsTeM908URCw0je0EsooOY22jamsGDWrv3B6kQa553w90tgtky7N%2Bu8gy5EaY0NhpYSrqYXfhCA04PkzhOy31KlDbTa%2Bp%2FZlf8Dw%3D%3D)


#### Planning Simulation
[Drawio link](https://viewer.diagrams.net/?tags=%7B%7D&highlight=0000ff&edit=_blank&layers=1&nav=1&title=psim_safety_island#R7V1bd5NKFP41fdTFcOex1lY9WvWc6rGely4aSIISiIS0jb%2F%2BDA0ksGc3EDozkKCupWG4hOz97fuemRPtbPbwJnHn08vY88MTVfEeTrTXJ6pKdE2n%2F2Ujq%2FWIRaz1wCQJvPyi7cBV8NvPB5V8dBl4%2FqJyYRrHYRrMq4OjOIr8UVoZc5Mkvq9eNo7D6rfO3YnPDFyN3JAd%2FRZ46XQ9ahvKdvytH0ymxTcTJT8zc4uL84HF1PXi%2B9KQdn6inSVxnK4%2FzR7O%2FDAjXkGX9X0XT5zdvFjiR2mTG6bpzY%2FEX315%2F%2Fvs%2B%2FLFezcYX794oRrrx9y54TL%2FxfnbpquCBEm8jDw%2Fewo50V7dT4PUv5q7o%2BzsPWU6HZumszA%2Fzb5V%2FqJ3fpL6D6Wh%2FC3f%2BPHMT5MVvSQ%2FqxYkzCGj6vnx%2FZYBpKDqtER8Mx9zc55PNo%2FekoV%2ByCmDU%2Bnv0cdLxfJOL9952qfV6o36r372guj7UEmRQiVHrVJJU1kqGSZCJYMHmXAwKZzBNA7C8CwO4%2BTxXm1sZH%2Bz8ThKS%2BPrP3R8kSbxT790xnz8w4fgRHGqFNdNluIbvVWmuGYLA2b%2FcGkD4WVppNsYKhVRNFIZGl26QURHzs%2B%2BMtSiPzKtkqSKqSiOfADMfMgNg0lED0eUdD4df5WRLKCG5DQ%2FMQs8L%2FsalAdVLvFQDwCrNssHjA2aKC5ovUeqqTSEqi4MqqyZuVreDgypJmHZYBkSkWoxTPBnfjLxo9HqkQPxnP4Xz%2F3ETSlVj8e%2B6cDtIoh5k%2Bt22QwnRvFsHCepe0vJPSReWE7XvHAaSIUZZhrpljLCnGSfBsAYBzEakmMT1ukeqpSoptE1M9hw%2BmpJCX8XLDLKH7kh16rs0DRENtD4SBg7TEQ2Ikrd7M7FfZCOptkjMuqNM9pwFY3xWB2NMBHwzFvT4BWUEmgpLIkK6fpXOPtk3V7fnCvfgzfWuTL9%2FKPw9Etk9L2Jf5UfUr00jSdx5Ibn21GAxe01H2Kqu9bk%2FeGn6SrPD7rLNK4S33MX0w2jnqTrIl4mI38XYHLnL3WpaUt3eeo59bKftpNPiR%2B6aXBXzSZiNH%2B89TRJ3FXpgnlMwbkoPflzNlBiv2pU2W9oRpmBe99AP6zfYcv%2FzY95hhyyntzCD8d0ZBZHgTCj9LTxecpcCTBKmkyBxKnP%2Bm6Dob7ZueOMJEWpwfcTqt0GwQHd6JwDbJZ0UBzAMiqSOaAOiwNOUVbrjQ1QG%2BRf%2Fcg7zSqmWUQRuotFMKqSuOqk%2BQ9Bep2fyT5%2Fz8ZfGvnR64fSZa9XxUGtI8jJjyt%2BXa0fVzh8tX5cuaqGli%2FWY%2Fu5e4x7ZkLvzAGAWP%2Fy%2FK4dfp4FdIChgwetKcM8aG8HlOhA3SuVKrQYf1JtUI89JjSTXoEU2pfWIGWCV1MUShXHQr9ILEob9FYcE0qbhsSHjlID5qs4odSEqtRWdr4YvL54r71%2FiFgpYHNwnKRgi%2FzvJZnApUB2fqjIPPZEGDr3K7ihia3OdqFTI%2ForNjdlB6W7ssPtbY9HzXXxM6BpstC8TrWP3n9fLxLr21ttrrqL36d20Q9zbMg0rOqDiCHIl4BfZCs1ahfeYErQuoWJKsnJnT8NRo8VwdHMo%2F9O3JRzxaPbmLu%2Be8FBYEwsABN%2BHU8NmvMkqiqlqqqs5%2BsqXjbVaWpT%2BxWrc3MwdagCBUVBFql%2BjwUbxmuuN6UoLrVfUvNSobCrSA7RVb6i83jzZz8JKO2yMv7ueny9PNl%2F5EmOPMFKR40jwPjO%2BbQCsfLEJn6PtvhHNNiB0nnevZj%2BJCD6bafObFOrqjNF0evUGaafpLkHBYAPTZ0dTcitsWnMYYUSWPOt5FCCf9ajbQajhfZ5hiujsLKPzjwrfJ6eyL4GG1NluzLcZJ9t3OrWfDk2MF82ZX0L89Uek02j1UJt9gSS%2FLxrs%2FogGz5IMCQLCehyfk5H3i1UBp339ejCgvV9ajv1rmhJgRhl9UFqVEd7LVFI%2F6EVijhWTaGegLMmeeXjtWqvFVGqtf36N4N3iAnEdWEdWFVZUZ4pK8%2FA%2FFAaAKRBuQbJJpyxuef1toxeLV1c%2FZ9DxZbYtclcvu5j0waCngmTQTgJkwGUr8CIBp2dxB%2BNLZL9LQLvJxFX72H0C0lsFxLUpo2hBLO9gtSyBf3uujw%2FuF51%2BE6xQnHdnwwRp8RwLa77lfJhe1nhoj9NYa3q0EobYmCt4S%2F8JKzB9bbK13vAE3sMrOXlnrtZZQpG%2BSaygpKwKB%2FlAf%2FApe8m0%2Bm3arHaWkyYu3OIIItp4y%2F8pGoB1zucG5NRWDewmIerRjbrrq0AhctaRNuBWe5apEEl4YDJDWwjul6PVHKzc74%2Fh24UBdGEPu6sWATjeBigQwYgS8FIZQCyLs9zzSbHvEdr21mb4EC6z3YBtCc2FvY62nCOT%2BNuDfMJxcs%2Fv4HDTljX8V4lmW3BxapUXJQa3HHKT%2B%2BKJQ6tJMO07UKnrS00TRgxiIam2jNo7oXM9hqxME31XSx%2FcCcEd%2BIqb20ssfLSsewK8l4qtrgmll2xz6E1sRyPlebfFsy1oU%2FRzd1gpgeddQQ3noTbr4ZgQgB69bYZYoheHTaeikZvgyLaAYeSIHWiGp2HksedqgK5E3QVV7n0ZnNVg0qeqNi0GbkcYNNXPXPZLMWU67Jh8%2FsPoXME%2BmzMZihtrZ7WcBkWXo1MiBa%2BizPOclUE3awajG0PghYZVdiuw2%2FzIDY7sCgtnQ1ofDSLZhNo%2FrDVGrFtnHgsmr28Pb0No%2FTTXx%2FIl5%2BmduksvV%2FonkIXU%2Fe3m3g3i9RNl4vj5YYJXXRkeX%2BpzMBSZhezZEb%2F3cx6uFnPebhYz3lgmyGOhjtwJiRBqmxSuYMt8DYYUYEVZqRPRSozsKA0F5XShhiDERa4OpzdNX%2BwIPYi347h5rGX62hZAUSFKNia2IpMZmDdGMNgBoHrhGE7%2BEjlBRbrDoQXsNlx02PaGTPQto2BcEMHbXh256KBdjO0Nurit5SEM6w0hIKbELq6p6QoCnKMICTQDzgtDqIPJNPvj49fyk0gO3HLzU2gTv5QuAHXe1bZQolcbqAu%2FUC4QVS9fpMPuexAnfqhsEMBfn337ED9%2BqHm7pg8d7HxXHepVdbT%2F%2BfTVTbyMfaOaTkzogC%2FnlgS90nFac%2F69e%2BizYZQ7yKqraLD3I8T1tJ7QGs0AhhERLtpJSkaGzov52CtqE14IT70IoBWGuJdosAVFnqhuzz1k1gWltCVSyw0Tu0FsVSwrE3jPdPFEQsNI3tBLKKDmNto2prBg1q7NgwpEWued8PdLILZMjyyXeWh0cBS0sXswmcacHqYxHFa7lOitJleUv8zu%2BJ%2F)

#### Planning Simulation with Single ECU
[Drawio link](https://viewer.diagrams.net/?tags=%7B%7D&highlight=0000ff&edit=_blank&layers=1&nav=1&title=psim_safety_island_one_ecu#R7V1bd5s4EP41Pqf7kBxA3PyYOrdum6TdtNtkX3IIyIYUkAs4ifPrV5iLQZIB2wIcu%2Bk5jRkDxvPNjL4ZjZQBGHmvF4Exta%2BQBd2BJFivA3A6kCRRBjL%2BFUvmiUQTtUQwCRwrPWkpuHXeYCoUUunMsWBYOjFCyI2caVloIt%2BHZlSSGUGAXsqnjZFb%2FtSpMYGU4NY0XFr607EiO5HqirCUX0JnYmefLArpO56RnZwKQtuw0EtBBM4GYBQgFCWvvNcRdGPlZXpJrjtf8W7%2BYAH0oyYXfDOvrwTNOrn6ZIGb%2BfxC%2BlceHaV3eTbcWfqF04eN5pkGAjTzLRjfRBiAjy%2B2E8HbqWHG775gzLHMjjwXH4n4Jf1Q2SfAIIKvBVH6kBcQeTAK5viU9F091VdqMJn6XpbaFxU5FdoF1Q%2B1VGikkE%2FyWy%2B1gl%2BkimEryY4engI4%2F%2F75bXQ%2FO%2FpsOOO7o6PsEZppSazX0thx3RFyUbC4FoyV%2BF8sR35UkCc%2FWB5GAfoFC%2B%2Boix8%2B%2BhYFQuOySqs8h6GocTDkoHGmWUqUwq8Mx8eSs9EPSvX4e0Zl%2FZb15SMfEkpPRYbrTHx8aGLtQSz%2FGGvNwc5%2Fkr7hOZYVfwwT0LJjcEGiDAQDB50FQ1soaBQK0IPBBPrmfIEAmuJfaAoDI8Ja3R%2BPkAkcRAYQIssh1LaQEOkIZCJvjILIeMT6PiQwJFXpGwyVAYaPdRBfGb44kWnHt4hDyjhWMVcsxmPJNFk6t9RHVeE1JJCRSNM79IC73653oz3ePZwJ986FdibYX5%2BOAKVGaGGilh5iR7DRBPmGe7aUEgF6ec4XhJ0lUe8TjKJ5yjqNWYTKyreM0M6BWqnXEM0CE1YZTBpIIwMHz6jixIzWxF%2BtEqcAukbkPJc5Kkvni0tPgsCYF06YImycYeHOX2NBAX5JKcOvAKUI4NoX4BfJMyzxz7%2FMFn6oU34YQnccc2%2FkO61FwdXRblV85OCRklxWMNB6H5KGlPatmectmAGBgvDhdvb41x6hQY5J6rBvNBgpSoYGZscwwFFviciHmEzvMx6y0jsedGK9Go9D8BCxd0TonLIOEUymn50QBfsEjAiIoTofWfpDRqaQocmeb53E1Tx8ZLpGGDpmWcdlqgdfnegufSd%2BfR%2FLj5X06PS1cNrpPDuopZOc2GB2Yi3JKwCiMPDIZOtxQYq7kZ6qkEWd5AulV1WRQKp8IRN3Svgvdae16akw1Jgf1CrblJTDstKmqch7t1KFLJ9xslKVYOyyLlQ%2BGHl%2B9lxrf5F2vYCufXDygqXl3xd8gu0FXeflWcVnR5xBJbPtTZ1BI02oYcTmZk10hXlba%2FLxE%2BVBNT4oRNX4cGlQi6PmcXWlmRWt5y4C19Z%2FP84D7eclmEpG%2BHaiZ5MZO2I8Yj6Vk4KuCxtaj6KVbyQqLY335AdlT7wyNJIXqB1ExmwYKdjyM7Qdc1GeNz0L%2Fz8xIs7V4F5TiSEBP2OWZMiwY5HHPC0bAkBBUFWSOoj0DrDmzztN78DuBPmMaqxBuLdgFwI9PjDbC7I69o4MEICccNuUXcgkTemYXch0RO58%2Fno3goDSe%2FVNpkPztkFgg7xhC5ZX6%2B7Z%2BL8jXtxewkzxRV40L50zXMHy2mFtcnv57DpVnRUDmqhLdUNafPQVBg5WRNzBtOWY1TQh3jFjz%2BPbtsauqJ0NWcwuB%2F7WuF69cGtmdSwVo7FQafUrzbS%2BHsM2v8Yln95yb6pmt3HuDco3Am0FZRGQbJBvZwebEVNu0F0q3U8PLsnY1C7TNjYGDVrDO2BsWUASjjWlFJPaK%2BgVg0iVeb63srFGjJL58dplY2KUVEkj5DdKsvUv7ZhprmWZm7OzjIzUVxT%2B2F0rdse%2FUWFrvqUKeinREAS52mrxAZkvdDap3HjCTdspAxZFwoKzudytLVgma%2BHcaJsGSh8kDvkm02z%2FaJC%2BvF%2BORs6bSQpjIQKosEL%2BFK1BZf0d65vMPFgLP7rVN91w%2FtU1fN%2FxJ%2Fh%2Bo2wNyP4gQLSkSKym824RoJvOtx2BtynXCcdDTS%2BP3Jqgtlayqwy6761mR7JCwGtMBQ0burgtQ6Gj8DOKkeUaCPpZ9yU3XffFZREqe%2FE1neyF%2BdQ9peP9WQtMDn%2BsDnqVgQQPIGaPJ4%2BuH938%2FUX8%2FksFV8OZ9Zu5VcC5bbwZgfUQRkY0C%2FcXDZVMAISewWBVQM69wMP%2F53PcD8kM93kyww0%2FJEvoR8yVJnuDFNnSKDKWZnWKFKtt%2FGDchkiZNEZVu1MwWAnTebp4%2B2Exs3AoUIgCa42c0CUYdDZFgiEcRswiGilVxoosqVNkWFnWYbgJNRGngb7dhLH3x8GgIRNbE%2BkM3tUxGkwanDCvwpYsG3Cv9nfWUgluBBjalFhba5FdMPy0%2BYcdVcf9bjM8Vv3%2BUNBQyJ7ZvlM8Zq2zJsfbX3ioeki2xUx%2FKTg9Lv9zcxtLrpG1T%2Bt8RIlc6MXiRG01J7F1T4%2FCn%2Fx8E4dPPg5W%2FvvceUsWd07XzFrTQfDPfMox34u077IfaxlFEyzaJ5cioSsgNdynrzVyyVyrviPKkgnupzSdaOGhrapVygVlTdO57YfQ8Wbunm0rSbo2K8ymrc78Nzh8Q0%2Fn6sfPd0fXP75dIlu%2FfDxl6D8h3guFJ9Q73uR68Q13DAhK6wxsVgJBLlTqdGc7JhKtLVVba6OXZY%2BBIGqDUo%2BBoMuDDXoMNmrzo5ZY1PQoVNl2fcN0r6szFKLYpG7ap6qSqUrDPtV1u%2Fx04nPkmn0RyPOVtJjFqymQif1ubO6Ve1PZk%2Br6bDk4ROXWpaW9Q1Z6zsE6BC8bbG2R50Y2eBR3aRNtY7pa2%2FLdW0iv3P51x2O6TqTTKtma2NSEdXLtQcMFd%2FUmjA%2BXfwQkOX35p1TA2f8%3D)

## Prerequisites
<a id="prerequisites"></a>
OS: Ubuntu22.04

ROS: ROS2 Humble

Git

(For single ECU planning simulation, high-performance machine with more then 8 threads. Example development environment: CPU Intel Core i7-9700K,  GPU RTX3070Ti.)

## Preparation
<a id="preparation"></a>
To understand the behavior, a planning simulation with a single ECU is recommended.

### Autoware
Please also look at [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/).

Currently MRM development is working on X2 micro bus project.

1. Clone pilot-auto.x2.
```
git clone https://github.com/tier4/pilot-auto.x2.git
```

2. Setup environment.
```
./setup-dev-env.sh
```

3. Get dependencies.
```
cd pilot-auto.x2
mkdir -p src
vcs import src < .beta.repo
```

4. Get ROS2 dependencies.
```
source /opt/ros/humble/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

5. Build.
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

6. Change Main ECU simple_planning_simulator launch parameters.
```
("input/ackermann_control_command", "/control_switch_interface/control_cmd"),
("input/gear_command", "/control_switch_interface/gear_cmd"),
("input/turn_indicators_command", "/control_switch_interface/turn_indicators_cmd"),
("input/hazard_lights_command", "/control_switch_interface/hazard_lights_cmd"),
```

7. Change Sub ECU topic namespace.
This is not required when operating simulation without Sub Autoware ECU.

### safety island
1. Clone Safety island.
```
git clone https://github.com/tier4/safety_island
```
2. Get dependencies.
```
cd safety_island
mkdir -p src
vcs import src < build_depends.repos
```

3. Get ROS2 dependencies.
```
source /opt/ros/humble/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

4. Build.
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --pakages-skip tilde_error_monitor tilde_aggregator external_watchdog
```


## How to Run
<a id="how-to-run"></a>
### Launch Safety Island
It is better to launch safety island before autoware.

#### Planning Simulation
```
cd safety_island
source install/setup.bash
ros2 launch launcher safety_island_psim.launch.xml
```

#### Planning Simulation with a Single ECU (recommended)
```
cd safety_island
source install/setup.bash
ros2 launch safety_island_psim_single_autoware_ecu.launch.xml
```

It is also possible to run only safety island on a different ECU from the Autoware ECU by using this configuration.

### Launch Autoware
Please also look at [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/planning-simulation/).
```
cd pilot-auto.x2
source install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=gsm8 sensor_model:=aip_x2
```


## Simulation Examples
<a id="simulation-examples"></a>

Currently in the main branch, configuration to achieve following behavior is already set. Also, some console output lines are included to help understanding the `supervisor` behavior.

### Planning Simulation with a Single ECU

Following conditions are different from normal run:

1. Main ECU control cmd and MRM operator result is also used as Sub ECU's.
1. `lane_departure` is treated as non self recoverable error.

#### Run Simulation
This simulation configuration is already done in main safety island branch.

1. Launch safety island on command-line 1.
    ```
    cd safety_island
    source install/setup.bash
    ros2 launch safety_island_psim_single_autoware_ecu.launch.xml
    ```

    At this point, the log of the command line 1 is as follows:

    ```
    ...
    [supervisor_psim-1] [INFO] [1690191340.471333997] [supervisor]: waiting for self_hazard_status_stamped msg of Main ...
    [supervisor_psim-1] [INFO] [1690191340.471361791] [supervisor]: waiting for mrm comfortable stop to become available on Main ...
    [supervisor_psim-1] [INFO] [1690191340.471368258] [supervisor]: waiting for mrm emergency stop to become available on Main ...
    [supervisor_psim-1] [INFO] [1690191340.471374802] [supervisor]: data not ready for supervisor
    ...
    ```

2. Launch Autoware on command-line 2.

    ```
    cd pilot-auto.x2
    source install/setup.bash
    ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=gsm8 sensor_model:=aip_x2
    ```

    At this point, the log of the command line 1 is as follows:

    ```
    ...
    [supervisor_psim-1] [INFO] [1690191469.237818330] [supervisor]: waiting for control_cmd to become available on Main ...
    [supervisor_psim-1] [INFO] [1690191469.237844626] [supervisor]: waiting for gear_cmd to become available on Main ...
    [supervisor_psim-1] [INFO] [1690191469.237851052] [supervisor]: waiting for turn_indicators_cmd_to become available on Main ...
    [supervisor_psim-1] [INFO] [1690191469.237856401] [supervisor]: waiting for hazard_lights_cmd_ to become available on Main ...
    [supervisor_psim-1] [INFO] [1690191470.971167992] [supervisor]: data not ready for supervisor
    ...
    ```

3. set initial pose and goal on command-line 3.
    ```
    cd safety_island/src/scripts
    chmod +x set_init_and_goal_supervisor_stop.sh
    ./set_init_and_goal_supervisor_stop.sh
    ```


    At this point, the log of the command line 1 is as follows:
    ```
    ...
    [supervisor_psim-1] [INFO] [1690191475.371165375] [supervisor]: Main is selected...
    [supervisor_psim-1] [INFO] [1690191476.371190188] [supervisor]: Main is selected...
    ...
    ```


4. start autonomous driving on command-line 3.
    ```
    cd safety_island/src/scripts
    chmod +x change_to_autonomous.sh
    ./change_to_autonomous.sh
    ```

    During autonomous driving, `voter` state transits as follows:

5. Normal -> Supervisor Emergency Stop
    At this point, the log of the command line 1 is as follows:
    ```
    ...
    [supervisor_psim-1] [INFO] [1690191580.104609290] [supervisor]: Main is selected...
    [supervisor_psim-1] Voter State changed: NORMAL -> SUPERVISOR_STOP
    [supervisor_psim-1] operateMrm SUPERVISOR_STOP
    [supervisor_psim-1] calling emergency stop on: Supervisor
    [supervisor_psim-1] [ERROR] [1690191580.481416839] [supervisor]: MRM Service did not respond on: Supervisor
    [supervisor_psim-1] Switch Status changed: Main -> Supervisor
    [supervisor_psim-1] operateMrm SUPERVISOR_STOP
    [supervisor_psim-1] operateMrm SUPERVISOR_STOP
    ...
    [supervisor_psim-1] [INFO] [1690191581.137820892] [supervisor]: Supervisor is selected...
    ...
    ```

    Here, the log tells us "MRM Service did not respond on: Supervisor". This does not mean a MRM stop operator did not start service. The MRM stop operator did not return response to `voter` even if the service was succesfully called, because of DDS implementation.
    The operation of the corresponding MRM stop operator is confirmed in the experimental environment.

6. Supervisor Emergency Stop -> Normal

    At this point, the log of the command line 1 is as follows:

    ```
    ...
    [supervisor_psim-1] Voter State changed: SUPERVISOR_STOP -> NORMAL
    [supervisor_psim-1] Switch Status changed: Supervisor -> Main
    [supervisor_psim-1] [ERROR] [1690191581.481416065] [supervisor]: MRM Service did not respond on: Supervisor
    [supervisor_psim-1] [INFO] [1690189904.756843401] [supervisor]: Main is selected...
    [supervisor_psim-1] [INFO] [1690189905.756884998] [supervisor]: Main is selected...
    ...
    ```

    Here, the MRM service is called to cancel a MRM operation.

7. Supervisor Emergency Stop -> Sub Comfortable Stop

    At this point, the log of the command line 1 is as follows:

    ```
    ...

    [supervisor_psim-1] Voter State changed: NORMAL -> SUPERVISOR_STOP
    [supervisor_psim-1] operateMrm SUPERVISOR_STOP
    [supervisor_psim-1] calling emergency stop on: Supervisor
    [supervisor_psim-1] [ERROR] [1690189922.000271712] [supervisor]: MRM Service did not respond on: Supervisor
    [supervisor_psim-1] Switch Status changed: Main -> Supervisor
    [supervisor_psim-1] operateMrm SUPERVISOR_STOP
    [supervisor_psim-1] [INFO] [1690189922.056764075] [supervisor]: Supervisor is selected...
    [supervisor_psim-1] operateMrm SUPERVISOR_STOP
    ...
    [supervisor_psim-1] Voter State changed: SUPERVISOR_STOP -> COMFORTABLE_STOP
    [supervisor_psim-1] operateMrm COMFORTABLE_STOP
    [supervisor_psim-1] [ERROR] [1690189922.566968354] [supervisor]: MRM Service did not respond on: Supervisor
    [supervisor_psim-1] [WARN] [1690189922.567591871] [supervisor]: comfortable_stop stop is called on: Sub
    [supervisor_psim-1] MRM called on Sub
    [supervisor_psim-1] Switch Status changed: Supervisor -> Sub
    [supervisor_psim-1] Voter State changed: COMFORTABLE_STOP -> MRM_SUCCEEDED
    [supervisor_psim-1] [INFO] [1690189923.056798595] [supervisor]: Sub is selected...
    [supervisor_psim-1] [INFO] [1690189924.056871367] [supervisor]: Sub is selected...
    ...
    ```

    Here, Comfortable Stop on the Sub ECU is called after Supervisor Emergency Stop.


### Simulation Configuration Examples

#### Set Non Self Recoverable Error
Autonomous running behavior can be changed by temporarily setting `lane_departure` error as non self recoverable error. To acheive this, add following parameters to `hazard_status_converter` config file `src/hazard_status_converter/config/hazard_status_converter.param.yaml` :

```
ros__parameters:
  non_self_recoverable_modules:
    /autoware/control/autonomous_driving/performance_monitoring/lane_departure: default
```

This setting change running behaviors as follows.

Before change: When `lane_departure` occurs, this error is judged as self recoverable error and `emergency_handler` run MRM Comfortable Stop on Main ECU.

After change: When `lane_departure` occurs, this error is judged as non self recoverable error and `voter` run MRM Emergency Stop on Supervisor ECU, then run MRM Comfortable Stop on Sub ECU.



#### Change Hazard Status Parameters

By changing `dummy_hazard_status_publisher` configuration file `src/dummy_hazard_status_publisher/config/dummy_hazard_status_publisher.param.yaml`, another behavior can be operated during autonomous driving.
Ex. Following setting cause MRM Comfortable Stop on Main 5 seconds after starting autonomous driving:


```
/**:
  ros__parameters:
    monitoring_topics:
      #/main/self_monitoring/hazard_status: default
      /sub/self_monitoring/hazard_status: default
      /supervisor/self_monitoring/hazard_status: default
      /main/external_monitoring/hazard_status/sub: {level: "3", emergency: "true", emergency_holding: "false", self_recoverable: "true", fault_time_after_engage: "5.0"}
      /main/external_monitoring/hazard_status/supervisor: default
      /sub/external_monitoring/hazard_status/main: default
      /sub/external_monitoring/hazard_status/supervisor: default
      /supervisor/external_monitoring/hazard_status/main: default
      /supervisor/external_monitoring/hazard_status/sub: {level: "3", emergency: "true", emergency_holding: "false", self_recoverable: "true", fault_time_after_engage: "5.0"}

```

This setting sets new parameters for `/main/external_monitoring/hazard_status/sub` and `supervisor/external_monitoring/hazard_status/sub`. The parameter `emergency` is set to `true` and `fault_time_after_engage` is set to `5.0`. This means 5 seconds after starting autonomous driving, two external monitoring results of Sub becomes emergency. By this, `voter` judges the Sub ECU has fault and operate MRM Comfortable Stop on Main. (If Main is already treated as a fault ECU in the `voter`, MRM Emergency Stop is continue operatiog on Supervisor and this monitoring result is ignored.)

cf. Reconfiguration of paramters above is available from command-line. Please see `dummy_hazard_status_publisher` README.

