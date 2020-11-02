## I. Introduction
Anytime bounded conflict-based search (ABCBS), based on related papers:
- [Suboptimal Variants of the Conflict-Based SearchAlgorithm for the Multi-Agent Pathfinding Problem](http://www.bgu.ac.il/~felner/2014/cbseShort.pdf) 
- [Anytime Focal Search with Applications](https://www.ijcai.org/Proceedings/2018/0199.pdf)

## II. File structure
```
.
├── ABCBS_logger.log                       # Log file
├── abcbs.py                               # Starter file
├── a_star.py
├── Environment.py
├── __init__.py
├── input.yaml                             # Input file
├── map_gene.ipynb
├── output.yaml                            # Output file
├── __pycache__
├── ReadMe.md
└── visualize.py                           # For visualization

```

## III. Arguments

### Argument explanations
- input: Path of input file                    
- output: Path of output file        
- wh: W_H in paper       
- wl: W_L in paper 
- gamma: Decay ratio for the W_H
- is_anytime: Whether to use anytime feature or not. If 'False', solution will be returned once ABCBS finds it   
- anytime_limitation: Time limitation for an anytime algorithm 
- is_logger_file: Whether to save logs to file. If 'False', logging displayed directly in the command line                         
- logger_file_path: Path for saving logs              
- logger_level: Level of logging, only 5 choices: "DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"        

### Default values of arguments
```
       anytime_limitation: 30                            
                    gamma: 0.985                         
                    input: input.yaml                    
               is_anytime: False                         
              logger_file: False                         
         logger_file_path: ABCBS_logger.log              
             logger_level: INFO                          
                   output: output.yaml                   
                       wh: 1.1                           
                       wl: 1.1        
```

## IV. Steps for running
### 1. Set input file
Example:
```
agents:                         # Information (start, goal and name) of each agent
-   start: [24, 26]
    goal: [2, 16]
    name: agent0
-   start: [31, 25]
    goal: [19, 30]
    name: agent1
map:
    dimensions: [32, 32]        # Dimension of the map
    obstacles:                  # Information (position) of obstacles. Note: type is tuple
    - !!python/tuple [14, 15]
    - !!python/tuple [9, 28]
    - !!python/tuple [5, 5]
```

### 2. Search paths
Easy to run:
```
python3 abcbs.py                # You can adjust arguments value according to section III
```
paths will be saved to `output.py`

### 3. Visualize paths
```
python3 input.yaml output.yaml
```

## V. Other
Code structure is based on [multi_agent_path_planning](https://github.com/atb033/multi_agent_path_planning)