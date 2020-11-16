### Pruning Trips

Refer to [SUMO route tools](https://sumo.dlr.de/docs/Tools/Routes.html) and look at the ```cutRoutes.py``` tool for pruning trips generated in a larger network for a new smaller network. The command used here is 
```
python cutRoutes.py {new network} {original routes} --orig-net {original net} --trips-output {output file} -v
```

*Note*: The routes file must contain ROUTES not TRIPS. To do this, DUAROUTER is used on the trip file to generate routes for each vehicle. This will serve as an approximate routing for each vehicle and determine which trips to keep for the smaller network. 

### Steps
1. Route original trips with DUAROUTER
    ```
    duarouter -n {original net} -t {original trips} -o {output file} --repair --routing-threads {num thread} --ignore-errors
    ```
2. Prune new network using ```simulation_net.py``` and a ```simulation net``` config (refer to ```simulation_net.cfg``` or ```mini_simulation_net.cfg```).
3. Use ```cutRoutes.py``` tool to prune trips for new network (as shown above).
4. Run the ```get_rid_of_none.py``` script to remove the None type vehicle (cutRoutes bug)
5. Validate trips using DUAROUTER again 
    ```
    duarouter -n {new network} -t {pruned trips} --repair --ignore-errors --write-trips -o {validated output file}
    ```