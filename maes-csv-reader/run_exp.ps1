# Copyright 2024 MAES
# 
# This file is part of MAES
# 
# MAES is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the
# Free Software Foundation, either version 3 of the License, or (at your option)
# any later version.
# 
# MAES is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
# Public License for more details.
# 
# You should have received a copy of the GNU General Public License along
# with MAES. If not, see http:#www.gnu.org/licenses/.
# 
# Contributors: Rasmus Borrisholt Schmidt, Andreas Sebastian Sørensen, Thor Beregaard
# 
# Original repository: https:#github.com/Molitany/MAES

#Testing different sizes with various amounts of robots - spawn together
##Global
###Together
python src/main.py -x Tick -y Tick `
    -d greed-size-100-global-robots-1-spawntogether greed-size-100-global-robots-3-spawntogether greed-size-100-global-robots-5-spawntogether greed-size-100-global-robots-7-spawntogether greed-size-100-global-robots-9-spawntogether tnf-size-100-global-robots-1-spawntogether tnf-size-100-global-robots-3-spawntogether tnf-size-100-global-robots-5-spawntogether tnf-size-100-global-robots-7-spawntogether tnf-size-100-global-robots-9-spawntogether minotaur-size-100-global-robots-1-spawntogether minotaur-size-100-global-robots-3-spawntogether minotaur-size-100-global-robots-5-spawntogether minotaur-size-100-global-robots-7-spawntogether minotaur-size-100-global-robots-9-spawntogether `
    -e all-strat-100-spawn-together-global -p false -t cactus

python src/main.py -x Tick -y Tick `
    -d greed-size-75-global-robots-1-spawntogether greed-size-75-global-robots-3-spawntogether greed-size-75-global-robots-5-spawntogether greed-size-75-global-robots-7-spawntogether greed-size-75-global-robots-9-spawntogether tnf-size-75-global-robots-1-spawntogether tnf-size-75-global-robots-3-spawntogether tnf-size-75-global-robots-5-spawntogether tnf-size-75-global-robots-7-spawntogether tnf-size-75-global-robots-9-spawntogether minotaur-size-75-global-robots-1-spawntogether minotaur-size-75-global-robots-3-spawntogether minotaur-size-75-global-robots-5-spawntogether minotaur-size-75-global-robots-7-spawntogether minotaur-size-75-global-robots-9-spawntogether `
    -e all-strat-75-spawn-together-global -p false -t cactus

python src/main.py -x Tick -y Tick `
    -d greed-size-50-global-robots-1-spawntogether greed-size-50-global-robots-3-spawntogether greed-size-50-global-robots-5-spawntogether greed-size-50-global-robots-7-spawntogether  greed-size-50-global-robots-9-spawntogether tnf-size-50-global-robots-1-spawntogether tnf-size-50-global-robots-3-spawntogether tnf-size-50-global-robots-5-spawntogether tnf-size-50-global-robots-7-spawntogether  tnf-size-50-global-robots-9-spawntogether minotaur-size-50-global-robots-1-spawntogether minotaur-size-50-global-robots-3-spawntogether minotaur-size-50-global-robots-5-spawntogether minotaur-size-50-global-robots-7-spawntogether  minotaur-size-50-global-robots-9-spawntogether `
    -e all-strat-50-spawn-together-global -p false -t cactus

###Apart
python src/main.py -x Tick -y Tick `
    -d greed-size-100-global-robots-1-spawnapart greed-size-100-global-robots-3-spawnapart greed-size-100-global-robots-5-spawnapart greed-size-100-global-robots-7-spawnapart greed-size-100-global-robots-9-spawnapart tnf-size-100-global-robots-1-spawnapart tnf-size-100-global-robots-3-spawnapart tnf-size-100-global-robots-5-spawnapart tnf-size-100-global-robots-7-spawnapart tnf-size-100-global-robots-9-spawnapart minotaur-size-100-global-robots-1-spawnapart minotaur-size-100-global-robots-3-spawnapart minotaur-size-100-global-robots-5-spawnapart minotaur-size-100-global-robots-7-spawnapart minotaur-size-100-global-robots-9-spawnapart `
    -e all-strat-100-spawn-apart-global -p false -t cactus

python src/main.py -x Tick -y Tick `
    -d greed-size-75-global-robots-1-spawnapart greed-size-75-global-robots-3-spawnapart greed-size-75-global-robots-5-spawnapart greed-size-75-global-robots-7-spawnapart greed-size-75-global-robots-9-spawnapart tnf-size-75-global-robots-1-spawnapart tnf-size-75-global-robots-3-spawnapart tnf-size-75-global-robots-5-spawnapart tnf-size-75-global-robots-7-spawnapart tnf-size-75-global-robots-9-spawnapart minotaur-size-75-global-robots-1-spawnapart minotaur-size-75-global-robots-3-spawnapart minotaur-size-75-global-robots-5-spawnapart minotaur-size-75-global-robots-7-spawnapart minotaur-size-75-global-robots-9-spawnapart `
    -e all-strat-75-spawn-apart-global -p false -t cactus

python src/main.py -x Tick -y Tick `
    -d greed-size-50-global-robots-1-spawnapart greed-size-50-global-robots-3-spawnapart greed-size-50-global-robots-5-spawnapart greed-size-50-global-robots-7-spawnapart  greed-size-50-global-robots-9-spawnapart tnf-size-50-global-robots-1-spawnapart tnf-size-50-global-robots-3-spawnapart tnf-size-50-global-robots-5-spawnapart tnf-size-50-global-robots-7-spawnapart  tnf-size-50-global-robots-9-spawnapart minotaur-size-50-global-robots-1-spawnapart minotaur-size-50-global-robots-3-spawnapart minotaur-size-50-global-robots-5-spawnapart minotaur-size-50-global-robots-7-spawnapart  minotaur-size-50-global-robots-9-spawnapart `
    -e all-strat-50-spawn-apart-global -p false -t cactus

##LOS
###Together
python src/main.py -x Tick -y Tick `
    -d greed-size-100-LOS-robots-1-spawntogether greed-size-100-LOS-robots-3-spawntogether greed-size-100-LOS-robots-5-spawntogether greed-size-100-LOS-robots-7-spawntogether greed-size-100-LOS-robots-9-spawntogether tnf-size-100-LOS-robots-1-spawntogether tnf-size-100-LOS-robots-3-spawntogether tnf-size-100-LOS-robots-5-spawntogether tnf-size-100-LOS-robots-7-spawntogether tnf-size-100-LOS-robots-9-spawntogether minotaur-size-100-LOS-robots-1-spawntogether minotaur-size-100-LOS-robots-3-spawntogether minotaur-size-100-LOS-robots-5-spawntogether minotaur-size-100-LOS-robots-7-spawntogether minotaur-size-100-LOS-robots-9-spawntogether `
    -e all-strat-100-spawn-together-LOS -p false -t cactus

python src/main.py -x Tick -y Tick `
    -d greed-size-75-LOS-robots-1-spawntogether greed-size-75-LOS-robots-3-spawntogether greed-size-75-LOS-robots-5-spawntogether greed-size-75-LOS-robots-7-spawntogether greed-size-75-LOS-robots-9-spawntogether tnf-size-75-LOS-robots-1-spawntogether tnf-size-75-LOS-robots-3-spawntogether tnf-size-75-LOS-robots-5-spawntogether tnf-size-75-LOS-robots-7-spawntogether tnf-size-75-LOS-robots-9-spawntogether minotaur-size-75-LOS-robots-1-spawntogether minotaur-size-75-LOS-robots-3-spawntogether minotaur-size-75-LOS-robots-5-spawntogether minotaur-size-75-LOS-robots-7-spawntogether minotaur-size-75-LOS-robots-9-spawntogether `
    -e all-strat-75-spawn-together-LOS -p false -t cactus

python src/main.py -x Tick -y Tick `
    -d greed-size-50-LOS-robots-1-spawntogether greed-size-50-LOS-robots-3-spawntogether greed-size-50-LOS-robots-5-spawntogether greed-size-50-LOS-robots-7-spawntogether greed-size-50-LOS-robots-9-spawntogether tnf-size-50-LOS-robots-1-spawntogether tnf-size-50-LOS-robots-3-spawntogether tnf-size-50-LOS-robots-5-spawntogether tnf-size-50-LOS-robots-7-spawntogether tnf-size-50-LOS-robots-9-spawntogether minotaur-size-50-LOS-robots-1-spawntogether minotaur-size-50-LOS-robots-3-spawntogether minotaur-size-50-LOS-robots-5-spawntogether minotaur-size-50-LOS-robots-7-spawntogether minotaur-size-50-LOS-robots-9-spawntogether `
    -e all-strat-50-spawn-together-LOS -p false -t cactus

###Apart
python src/main.py -x Tick -y Tick `
    -d greed-size-100-LOS-robots-1-spawnapart greed-size-100-LOS-robots-3-spawnapart greed-size-100-LOS-robots-5-spawnapart greed-size-100-LOS-robots-7-spawnapart greed-size-100-LOS-robots-9-spawnapart tnf-size-100-LOS-robots-1-spawnapart tnf-size-100-LOS-robots-3-spawnapart tnf-size-100-LOS-robots-5-spawnapart tnf-size-100-LOS-robots-7-spawnapart tnf-size-100-LOS-robots-9-spawnapart minotaur-size-100-LOS-robots-1-spawnapart minotaur-size-100-LOS-robots-3-spawnapart minotaur-size-100-LOS-robots-5-spawnapart minotaur-size-100-LOS-robots-7-spawnapart minotaur-size-100-LOS-robots-9-spawnapart `
    -e all-strat-100-spawn-apart-LOS -p false -t cactus

python src/main.py -x Tick -y Tick `
    -d  greed-size-75-LOS-robots-1-spawnapart greed-size-75-LOS-robots-3-spawnapart greed-size-75-LOS-robots-5-spawnapart greed-size-75-LOS-robots-7-spawnapart greed-size-75-LOS-robots-9-spawnapart tnf-size-75-LOS-robots-1-spawnapart tnf-size-75-LOS-robots-3-spawnapart tnf-size-75-LOS-robots-5-spawnapart tnf-size-75-LOS-robots-7-spawnapart tnf-size-75-LOS-robots-9-spawnapart minotaur-size-75-LOS-robots-1-spawnapart minotaur-size-75-LOS-robots-3-spawnapart minotaur-size-75-LOS-robots-5-spawnapart minotaur-size-75-LOS-robots-7-spawnapart minotaur-size-75-LOS-robots-9-spawnapart `
    -e all-strat-75-spawn-apart-LOS -p false -t cactus

python src/main.py -x Tick -y Tick `
    -d greed-size-50-LOS-robots-1-spawnapart greed-size-50-LOS-robots-3-spawnapart greed-size-50-LOS-robots-5-spawnapart greed-size-50-LOS-robots-7-spawnapart  greed-size-50-LOS-robots-9-spawnapart tnf-size-50-LOS-robots-1-spawnapart tnf-size-50-LOS-robots-3-spawnapart tnf-size-50-LOS-robots-5-spawnapart tnf-size-50-LOS-robots-7-spawnapart  tnf-size-50-LOS-robots-9-spawnapart minotaur-size-50-LOS-robots-1-spawnapart minotaur-size-50-LOS-robots-3-spawnapart minotaur-size-50-LOS-robots-5-spawnapart minotaur-size-50-LOS-robots-7-spawnapart  minotaur-size-50-LOS-robots-9-spawnapart `
    -e all-strat-50-spawn-apart-LOS -p false -t cactus

##Material
###Together
python src/main.py -x Tick -y Tick `
    -d greed-size-50-material-robots-1-spawntogether greed-size-50-material-robots-3-spawntogether greed-size-50-material-robots-5-spawntogether greed-size-50-material-robots-7-spawntogether greed-size-50-material-robots-9-spawntogether tnf-size-50-material-robots-1-spawntogether tnf-size-50-material-robots-3-spawntogether tnf-size-50-material-robots-5-spawntogether tnf-size-50-material-robots-7-spawntogether tnf-size-50-material-robots-9-spawntogether minotaur-size-50-material-robots-1-spawntogether minotaur-size-50-material-robots-3-spawntogether minotaur-size-50-material-robots-5-spawntogether minotaur-size-50-material-robots-7-spawntogether minotaur-size-50-material-robots-9-spawntogether `
    -e all-strat-50-spawn-together-material -p false -t cactus

python src/main.py -x Tick -y Tick `
    -d greed-size-75-material-robots-1-spawntogether greed-size-75-material-robots-3-spawntogether greed-size-75-material-robots-5-spawntogether greed-size-75-material-robots-7-spawntogether greed-size-75-material-robots-9-spawntogether tnf-size-75-material-robots-1-spawntogether tnf-size-75-material-robots-3-spawntogether tnf-size-75-material-robots-5-spawntogether tnf-size-75-material-robots-7-spawntogether tnf-size-75-material-robots-9-spawntogether minotaur-size-75-material-robots-1-spawntogether minotaur-size-75-material-robots-3-spawntogether minotaur-size-75-material-robots-5-spawntogether minotaur-size-75-material-robots-7-spawntogether minotaur-size-75-material-robots-9-spawntogether `
    -e all-strat-75-spawn-together-material -p false -t cactus

python src/main.py -x Tick -y Tick `
    -d greed-size-100-material-robots-1-spawntogether greed-size-100-material-robots-3-spawntogether greed-size-100-material-robots-5-spawntogether greed-size-100-material-robots-7-spawntogether greed-size-100-material-robots-9-spawntogether tnf-size-100-material-robots-1-spawntogether tnf-size-100-material-robots-3-spawntogether tnf-size-100-material-robots-5-spawntogether tnf-size-100-material-robots-7-spawntogether tnf-size-100-material-robots-9-spawntogether minotaur-size-100-material-robots-1-spawntogether minotaur-size-100-material-robots-3-spawntogether minotaur-size-100-material-robots-5-spawntogether minotaur-size-100-material-robots-7-spawntogether minotaur-size-100-material-robots-9-spawntogether `
    -e all-strat-100-spawn-together-material -p false -t cactus

###Apart
python src/main.py -x Tick -y Tick `
    -d greed-size-100-material-robots-1-spawnapart greed-size-100-material-robots-3-spawnapart greed-size-100-material-robots-5-spawnapart greed-size-100-material-robots-7-spawnapart greed-size-100-material-robots-9-spawnapart tnf-size-100-material-robots-1-spawnapart tnf-size-100-material-robots-3-spawnapart tnf-size-100-material-robots-5-spawnapart tnf-size-100-material-robots-7-spawnapart tnf-size-100-material-robots-9-spawnapart minotaur-size-100-material-robots-1-spawnapart minotaur-size-100-material-robots-3-spawnapart minotaur-size-100-material-robots-5-spawnapart minotaur-size-100-material-robots-7-spawnapart minotaur-size-100-material-robots-9-spawnapart `
    -e all-strat-100-spawn-apart-material -p false -t cactus

python src/main.py -x Tick -y Tick `
    -d greed-size-75-material-robots-1-spawnapart greed-size-75-material-robots-3-spawnapart greed-size-75-material-robots-5-spawnapart greed-size-75-material-robots-7-spawnapart greed-size-75-material-robots-9-spawnapart tnf-size-75-material-robots-1-spawnapart tnf-size-75-material-robots-3-spawnapart tnf-size-75-material-robots-5-spawnapart tnf-size-75-material-robots-7-spawnapart tnf-size-75-material-robots-9-spawnapart minotaur-size-75-material-robots-1-spawnapart minotaur-size-75-material-robots-3-spawnapart minotaur-size-75-material-robots-5-spawnapart minotaur-size-75-material-robots-7-spawnapart minotaur-size-75-material-robots-9-spawnapart `
    -e all-strat-75-spawn-apart-material -p false -t cactus

python src/main.py -x Tick -y Tick `
    -d greed-size-50-material-robots-1-spawnapart greed-size-50-material-robots-3-spawnapart greed-size-50-material-robots-5-spawnapart greed-size-50-material-robots-7-spawnapart  greed-size-50-material-robots-9-spawnapart tnf-size-50-material-robots-1-spawnapart tnf-size-50-material-robots-3-spawnapart tnf-size-50-material-robots-5-spawnapart tnf-size-50-material-robots-7-spawnapart  tnf-size-50-material-robots-9-spawnapart minotaur-size-50-material-robots-1-spawnapart minotaur-size-50-material-robots-3-spawnapart minotaur-size-50-material-robots-5-spawnapart minotaur-size-50-material-robots-7-spawnapart  minotaur-size-50-material-robots-9-spawnapart `
    -e all-strat-50-spawn-apart-material -p false -t cactus

##None
###Together
python src/main.py -x Tick -y Tick `
    -d greed-size-50-none-robots-1-spawntogether greed-size-50-none-robots-3-spawntogether greed-size-50-none-robots-5-spawntogether greed-size-50-none-robots-7-spawntogether greed-size-50-none-robots-9-spawntogether tnf-size-50-none-robots-1-spawntogether tnf-size-50-none-robots-3-spawntogether tnf-size-50-none-robots-5-spawntogether tnf-size-50-none-robots-7-spawntogether tnf-size-50-none-robots-9-spawntogether minotaur-size-50-none-robots-1-spawntogether minotaur-size-50-none-robots-3-spawntogether minotaur-size-50-none-robots-5-spawntogether minotaur-size-50-none-robots-7-spawntogether minotaur-size-50-none-robots-9-spawntogether `
    -e all-strat-50-spawn-together-none -p false -t cactus

python src/main.py -x Tick -y Tick `
    -d greed-size-75-none-robots-1-spawntogether greed-size-75-none-robots-3-spawntogether greed-size-75-none-robots-5-spawntogether greed-size-75-none-robots-7-spawntogether greed-size-75-none-robots-9-spawntogether tnf-size-75-none-robots-1-spawntogether tnf-size-75-none-robots-3-spawntogether tnf-size-75-none-robots-5-spawntogether tnf-size-75-none-robots-7-spawntogether tnf-size-75-none-robots-9-spawntogether minotaur-size-75-none-robots-1-spawntogether minotaur-size-75-none-robots-3-spawntogether minotaur-size-75-none-robots-5-spawntogether minotaur-size-75-none-robots-7-spawntogether minotaur-size-75-none-robots-9-spawntogether `
    -e all-strat-75-spawn-together-none -p false -t cactus

python src/main.py -x Tick -y Tick `
    -d greed-size-100-none-robots-1-spawntogether greed-size-100-none-robots-3-spawntogether greed-size-100-none-robots-5-spawntogether greed-size-100-none-robots-7-spawntogether greed-size-100-none-robots-9-spawntogether tnf-size-100-none-robots-1-spawntogether tnf-size-100-none-robots-3-spawntogether tnf-size-100-none-robots-5-spawntogether tnf-size-100-none-robots-7-spawntogether tnf-size-100-none-robots-9-spawntogether minotaur-size-100-none-robots-1-spawntogether minotaur-size-100-none-robots-3-spawntogether minotaur-size-100-none-robots-5-spawntogether minotaur-size-100-none-robots-7-spawntogether minotaur-size-100-none-robots-9-spawntogether `
    -e all-strat-100-spawn-together-none -p false -t cactus

###Apart
python src/main.py -x Tick -y Tick `
    -d greed-size-100-none-robots-1-spawnapart greed-size-100-none-robots-3-spawnapart greed-size-100-none-robots-5-spawnapart greed-size-100-none-robots-7-spawnapart greed-size-100-none-robots-9-spawnapart tnf-size-100-none-robots-1-spawnapart tnf-size-100-none-robots-3-spawnapart tnf-size-100-none-robots-5-spawnapart tnf-size-100-none-robots-7-spawnapart tnf-size-100-none-robots-9-spawnapart minotaur-size-100-none-robots-1-spawnapart minotaur-size-100-none-robots-3-spawnapart minotaur-size-100-none-robots-5-spawnapart minotaur-size-100-none-robots-7-spawnapart minotaur-size-100-none-robots-9-spawnapart `
    -e all-strat-100-spawn-apart-none -p false -t cactus

python src/main.py -x Tick -y Tick `
    -d greed-size-75-none-robots-1-spawnapart greed-size-75-none-robots-3-spawnapart greed-size-75-none-robots-5-spawnapart greed-size-75-none-robots-7-spawnapart greed-size-75-none-robots-9-spawnapart tnf-size-75-none-robots-1-spawnapart tnf-size-75-none-robots-3-spawnapart tnf-size-75-none-robots-5-spawnapart tnf-size-75-none-robots-7-spawnapart tnf-size-75-none-robots-9-spawnapart minotaur-size-75-none-robots-1-spawnapart minotaur-size-75-none-robots-3-spawnapart minotaur-size-75-none-robots-5-spawnapart minotaur-size-75-none-robots-7-spawnapart minotaur-size-75-none-robots-9-spawnapart `
    -e all-strat-75-spawn-apart-none -p false -t cactus

python src/main.py -x Tick -y Tick `
    -d greed-size-50-none-robots-1-spawnapart greed-size-50-none-robots-3-spawnapart greed-size-50-none-robots-5-spawnapart greed-size-50-none-robots-7-spawnapart  greed-size-50-none-robots-9-spawnapart tnf-size-50-none-robots-1-spawnapart tnf-size-50-none-robots-3-spawnapart tnf-size-50-none-robots-5-spawnapart tnf-size-50-none-robots-7-spawnapart  tnf-size-50-none-robots-9-spawnapart minotaur-size-50-none-robots-1-spawnapart minotaur-size-50-none-robots-3-spawnapart minotaur-size-50-none-robots-5-spawnapart minotaur-size-50-none-robots-7-spawnapart  minotaur-size-50-none-robots-9-spawnapart `
    -e all-strat-50-spawn-apart-none -p false -t cactus
