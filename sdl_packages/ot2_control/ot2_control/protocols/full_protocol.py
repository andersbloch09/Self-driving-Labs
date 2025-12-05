from opentrons import protocol_api
import time
import heapq

metadata = {
    "protocolName": "Fehlings test full protocol",
    "description": """This protocol is made to perform a water bath scheduling test.""",
    "author": "Emil uStabil"
}

class Timer:
    def __init__(self):
        self.start_time = None
        self.running = False

    def start(self):
        """Start the timer"""
        if not self.running:
            self.start_time = time.perf_counter()
            self.running = True

    def elapsed(self):
        """Get the elapsed time in seconds"""
        if self.running:
            return time.perf_counter() - self.start_time
        return 0

    def reset(self):
        """Reset the timer"""
        self.start_time = None
        self.running = False


def generate_concentrations(max_conc, min_conc, n, dtype="linear"):
    """
    Generate n concentrations between max_conc and min_conc (inclusive).

    dtype: "linear" or "log"
    Returns a list of floats (descending from max to min).
    """
    if n < 1:
        raise ValueError("n must be >= 1")
    if n == 1:
        return [float(max_conc)]

    dtype = dtype.lower()
    if dtype not in ("linear", "log"):
        raise ValueError('dtype must be "linear" or "log"')

    max_c = float(max_conc)
    min_c = float(min_conc)

    if dtype == "linear":
        step = (min_c - max_c) / (n - 1)
        return [max_c + i * step for i in range(n)]

    # log (geometric) spacing
    if max_c <= 0 or min_c <= 0:
        raise ValueError("For log spacing, both concentrations must be > 0")
    #logarithmic method
    ratio = (min_c / max_c) ** (1.0 / (n - 1))
    result = []
    for i in range(n):
        result.append(max_c * (ratio ** i))
    return result

def current_time():
    return time.time()

def wait_until(time_target):
    while current_time() < time_target:

        time.sleep(0.5)



requirements = {"robotType": "OT-2", "apiLevel": "2.20"}

def add_parameters(parameters: protocol_api.ParameterContext):

    parameters.add_int(
        variable_name="sample_count",
        display_name="Sample Count",
        description="Number of samples to process.",
        default=1,
        minimum=1,
        maximum=24
    )

    parameters.add_float(
        variable_name="v_f",
        display_name="Sample Volume (mL)",
        description="Final volume of each sample solution.",
        default=1000,
        minimum=100,
        maximum=3000
    )

    parameters.add_float(
        variable_name="C_stock",
        display_name="Stock Concentration (%)",
        description="Concentration of the stock solution.",
        default=0.3,
        minimum=0.0,
        maximum=0.3
    )

    parameters.add_float(
        variable_name="heating_time",
        display_name="Heating Time (s)",
        description="Time to heat samples in the water bath.",
        default=300,
        minimum=60,
        maximum=600
    )

    parameters.add_int(
        variable_name="large_tips_used",
        display_name="Large Tips Used",
        description="Number of large tips used from the tip rack.",
        default=0,
        minimum=0,
        maximum=96
    )

    parameters.add_int(
        variable_name="small_tips_used",
        display_name="Small Tips Used",
        description="Number of small tips used from the tip rack.",
        default=0,
        minimum=0,
        maximum=96
    )

    parameters.add_float(
        variable_name="max_concentration",
        display_name="Max Concentration (%)",
        description="Maximum concentration for dilution series.",
        default=0.3,
        minimum=0.0,
        maximum=0.3
    )

    parameters.add_float(
        variable_name="min_concentration",
        display_name="Min Concentration (%)",
        description="Minimum concentration for dilution series.",
        default=0.3,
        minimum=0.0,
        maximum=0.3
    )

    parameters.add_str(
        variable_name="dilution_method",
        display_name="Dilution Method",
        description="Method of dilution: linear or log.",
        choices=[
            {"display_name":"Linear","value":"linear"},
            {"display_name":"Logarithmic","value":"log"}],
        default="linear"
    )

    parameters.add_float(
        variable_name="v_stock_start",
        display_name="Initial Stock Volume (µL)",
        description="Initial volume of stock solution available.",
        default=10000,
        minimum=1000,
        maximum=100000
    )

    parameters.add_float(
        variable_name="v_water_start",
        display_name="Initial Water Volume (µL)",
        description="Initial volume of water available.",
        default=10000,
        minimum=1000,
        maximum=100000
    )

    parameters.add_float(
        variable_name="v_container",
        display_name="Container Volume (µL)",
        description="Volume of the ingredient container.",
        default=10000,
        minimum=1000,
        maximum=100000
    )

def run(protocol: protocol_api.ProtocolContext):
    
    tt = Timer()
    tt.start()
    #defining the cuvette well names for transfer tasks
    
    cuvette_tip_wells = ["A1", "C1", "E1", "G1",
                         "A3", "C3", "E3", "G3",
                         "A5", "C5", "E5", "G5",
                         "A7", "C7", "E7", "G7",
                         "A9", "C9", "E9", "G9",
                         "A11", "C11", "E11", "G11"]
    
    water_bath_wells = ["A1", "A5", "A9", "A13","A17"]
    
    #loading deck labware and positions

    protocol.comment("Processing " + str(protocol.params.sample_count) + " samples.")

    large_tips = protocol.load_labware("opentrons_96_tiprack_300ul", 9)
    large_tips.set_offset(x=0.5, y=-0.3, z=0.0)

    cuvette_rack = protocol.load_labware("aau_24_tuberack_3000ul", 1)
    cuvette_rack.set_offset(x=-0.7, y=-1.7, z=6.0)

    sample_ingredients = protocol.load_labware("aau_24_tuberack_10000ul", 3)
    sample_ingredients.set_offset(x=-0.7, y=-0.9, z=6.0)

    fehlings_ingredients = protocol.load_labware("aau_24_tuberack_10000ul", 2)
    fehlings_ingredients.set_offset(x=-0.7, y=-1.0, z=6.0)

    water_bath = protocol.load_labware("aau_144_tiprack_3000ul", 10)
    water_bath.set_offset(x=1.0, y=-1.4, z=1.7) 

    fehlings_mix = protocol.load_labware("aau_24_tuberack_3000ul", 8)
    fehlings_mix.set_offset(x=0.0, y=-0.5, z=0.0)

    samples = protocol.load_labware("aau_24_tuberack_3000ul", 11)
    samples.set_offset(x=0.5, y=-0.8, z=0.0)

    #loading pipettes
    right_pipette = protocol.load_instrument("p300_single_gen2", "right", tip_racks=[large_tips])

    right_pipette.starting_tip = large_tips.wells()[protocol.params.large_tips_used]

    #initialize pipette tip counters
    small_pipettes = int(0)
    large_pipettes = int(0)

  
    v_fehlings = protocol.params.v_f * 0.15
    v_AB = (v_fehlings + 50)/2
    
    #calculate volumes needed for each concentration

    c1 = protocol.params.C_stock
    v_f = protocol.params.v_f
    c2_list = generate_concentrations(protocol.params.max_concentration,
                                     protocol.params.min_concentration,
                                     protocol.params.sample_count,
                                     dtype=protocol.params.dilution_method)

    protocol.comment("Target concentrations: " + ", ".join([f"{c:.2f}%" for c in c2_list]))

    if len(c2_list) != protocol.params.sample_count:
        raise ValueError("Number of concentrations provided does not match sample count.")

    v_stock = []
    v_water = []
    stock_used = 0.0
    water_used = 0.0

    sample_count = protocol.params.sample_count


    for i in range(sample_count):
        c_stock = c2_list[i]
        v_stock_value = (c_stock * v_f) / c1
        v_stock.append(v_stock_value)
        v_water.append(v_f - v_stock_value)

    #prepare samples with calculated volumes
    for i in range(sample_count):
        if v_stock[i] > 0:
            #adding glucose stock solution to sample wells
            right_pipette.transfer(v_stock[i], fehlings_ingredients["B1"], samples.wells()[i])
            large_pipettes += 1
            stock_used += v_stock[i]
        
        if v_water[i] > 0:
            #adding water to sample wells
            right_pipette.transfer(v_water[i], fehlings_ingredients["B2"], samples.wells()[i])
            large_pipettes += 1
            water_used += v_water[i]
        
    
     #water bath scheduling algorithm initialization
    waterbath_slots = len(water_bath_wells)
    slots = [None] * waterbath_slots
    removal_queue = []
    next_slot = 0
    fehlings_a_used = 0.0
    fehlings_b_used = 0.0

    prep_times = []

    for item_id in range(protocol.params.sample_count):
        
        del protocol.deck["1"]

        cuvette_for_transfer = protocol.load_labware("aau_96_tiprack_3000ul", 1)
        cuvette_for_transfer.set_offset(x=-0.5, y=-1.5, z=6.0)

        while True:

            # check for any close expirations
            if removal_queue:
                soonest_remove_time, _, _ = removal_queue[0]
                time_to_expiry = soonest_remove_time - current_time()
                if time_to_expiry < 130:  #130 seconds is the rough time it takes to prepare a new sample
                    wait_until(soonest_remove_time)
                    protocol.comment("Waiting for sample to finish heating...")


            # 1. Check for expired items and free their slots
            now = current_time()
            while removal_queue and removal_queue[0][0] <= now:
                _, expired_id, expired_slot = heapq.heappop(removal_queue)
                right_pipette.pick_up_tip(water_bath[water_bath_wells[expired_slot]])
                right_pipette.drop_tip(cuvette_for_transfer[cuvette_tip_wells[expired_id]])
                slots[expired_slot] = None

            

            # 2. If next slot is free, place item
            free_slots = [i for i, s in enumerate(slots) if s is None]
            if free_slots:
                slot_to_use = free_slots[0]
                
                del protocol.deck["1"]

                cuvette_rack = protocol.load_labware("aau_24_tuberack_3000ul", 1)
                cuvette_rack.set_offset(x=-0.7, y=-1.7, z=6.0)

                #prepare next test sample
                t = Timer()
                t.start()

                right_pipette.transfer(v_AB, fehlings_ingredients["B5"], fehlings_mix.wells()[item_id]) #aspirating and dispensing A part
                large_pipettes += 1
                fehlings_a_used += v_AB

                right_pipette.pick_up_tip()
                large_pipettes += 1
                right_pipette.aspirate(v_AB, fehlings_ingredients["B6"]) #aspirating B part
                fehlings_b_used += v_AB
                right_pipette.dispense(v_AB, fehlings_mix.wells()[item_id]) #dispensing B part
                right_pipette.mix(2, v_fehlings + 50, fehlings_mix.wells()[item_id]) # mixing the fehlings solution
                
                right_pipette.aspirate(v_fehlings, fehlings_mix.wells()[item_id])
                right_pipette.dispense(v_fehlings, samples.wells()[item_id])
                right_pipette.mix(3, v_fehlings, samples.wells()[item_id])
                right_pipette.blow_out()
                

                right_pipette.transfer(
                    protocol.params.v_f,                 # e.g. 1000 µL
                    samples.wells()[item_id],
                    cuvette_rack.wells()[item_id].top(-10),
                    new_tip='never'                      # <-- REUSE THE SAME TIP
                )

                right_pipette.drop_tip()
                prep_time = t.elapsed()
                prep_times.append(prep_time)


                del protocol.deck["1"]

                cuvette_for_transfer = protocol.load_labware("aau_96_tiprack_3000ul", 1)
                cuvette_for_transfer.set_offset(x=-0.5, y=-1.5, z=6.0)

                right_pipette.pick_up_tip(cuvette_for_transfer[cuvette_tip_wells[item_id]])
                right_pipette.drop_tip(water_bath[water_bath_wells[slot_to_use]])

                removal_time = current_time() + protocol.params.heating_time
                slots[slot_to_use] = (item_id, removal_time)
                heapq.heappush(removal_queue, (removal_time, item_id, slot_to_use))

                next_slot = (next_slot + 1) % waterbath_slots
                break  # go to next item


            else:
                # 3. Otherwise: no free slot → wait until the earliest one frees
                soonest_remove_time, soonest_id, soonest_slot = removal_queue[0]
                protocol.comment(f"No space—robot waits {soonest_remove_time - current_time()} until slot {soonest_slot + 1} is free...")
                wait_until(soonest_remove_time)

    while removal_queue:
        removal_time, expired_id, expired_slot = heapq.heappop(removal_queue)
        wait_until(removal_time)
        right_pipette.pick_up_tip(water_bath[water_bath_wells[expired_slot]])
        right_pipette.drop_tip(cuvette_for_transfer[cuvette_tip_wells[expired_id]])


    protocol.comment("No. of samples:" + str(protocol.params.sample_count))
    protocol.comment("Target concentrations[%]: " + ", ".join([f"{c:.2f}" for c in c2_list]))
    protocol.comment("Large pipette tips used: " + str(large_pipettes))
    protocol.comment("Small pipette tips used: " + str(small_pipettes))
    protocol.comment(f"Glucose stock solution used [ul]: {stock_used:.2f}")
    protocol.comment(f"Water used [ul]: {water_used:.2f}")
    protocol.comment(f"Fehlings A solution used [ul]: {fehlings_a_used:.2f}")
    protocol.comment(f"Fehlings B solution used [ul]: {fehlings_b_used:.2f}")
    protocol.comment("Prep times[s]: " + ", ".join([f"{pt:.2f}" for pt in prep_times]))

    total_time = tt.elapsed()
    protocol.comment(f"Total protocol time[s]: {total_time:.2f}")