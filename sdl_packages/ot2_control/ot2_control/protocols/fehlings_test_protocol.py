from opentrons import protocol_api
import time

metadata = {
    "protocolName": "Fehlings test protocol",
    "description": """This protocol is made to perform a fehlings test.""",
    "author": "Emil uStabil"
}

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
        display_name="Stock Concentration (mM)",
        description="Concentration of the stock solution.",
        default=100,
        minimum=10,
        maximum=1000
    )

    parameters.add_csv_file(
        variable_name="concentration_list",
        display_name="Concentration List",
        description="CSV file containing desired concentrations for each sample (mM).",
        #default="10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10"
    )



def run(protocol: protocol_api.ProtocolContext):
    
    
    cuvette_tip_wells = ["A1", "C1", "E1", "G1",
                         "A2", "C2", "E2", "G2",
                         "A3", "C3", "E3", "G3",
                         "A4", "C4", "E4", "G4",
                         "A5", "C5", "E5", "G5",
                         "A6", "C6", "E6", "G6"]
    
    
    
    #load deck

    protocol.comment("Processing " + str(protocol.params.sample_count) + " samples.")

    large_tips = protocol.load_labware("opentrons_96_tiprack_300ul", 10)
    cuvette_rack = protocol.load_labware("aau_24_tuberack_3000ul", 1)
    sample_ingredients = protocol.load_labware("corning_6_wellplate_16.8ml_flat", 2) #placeholder right now
    fehlings_ingredients = protocol.load_labware("corning_6_wellplate_16.8ml_flat", 3) #placeholder right now
    water_bath = protocol.load_labware("aau_96_tiprack_3000ul", 4) #placeholder right now
    fehlings_mix = protocol.load_labware("aau_24_tuberack_3000ul", 6) #placeholder right now

    right_pipette = protocol.load_instrument("p300_single_gen2", "right", tip_racks=[large_tips])

    #prepare test samples
        #mix 1ml solutions of water and sugar in sample tubes in required concentrations (remember control tube)
    
    for i in range(protocol.params.sample_count):
        right_pipette.transfer(protocol.params.v_f, sample_ingredients["A1"], cuvette_rack.wells()[i].top(z=-10)) #placeholder step, need to calculate correct volumes based on desired concentration

    protocol.comment("Samples prepared.")

    #preparing fehlings solution
        #mix part a and part b in container
    
    """
    for i in range(protocol.params.sample_count):
        del protocol.deck["1"]

        cuvette_rack = protocol.load_labware("aau_24_tuberack_3000ul", 1)

        right_pipette.transfer(300, fehlings_ingredients["A1"], fehlings_mix.wells()[i]) #aspirating and dispensing A part
        right_pipette.pick_up_tip()
        right_pipette.aspirate(300, fehlings_ingredients["A2"]) #aspirating B part
        right_pipette.dispense(300, fehlings_mix.wells()[i]) #dispensing B part
        right_pipette.mix(3, 300, fehlings_mix.wells()[i]) # mixing the fehlings solution
        # we could add a touch tip here but probably not necessary


        #testing samples with fehlings method
        #add 2-3 drops of fehlings solution to each sample tube
        right_pipette.aspirate(50, fehlings_mix.wells()[i])
        right_pipette.dispense(50, cuvette_rack.wells()[i].top(z=-10))
        right_pipette.blow_out()
        right_pipette.drop_tip()
        #maybe add mixing here?

        #move all tubes to water bath and heat for 1-2 minutes
        del protocol.deck["1"]

        cuvette_for_transfer = protocol.load_labware("aau_96_tiprack_3000ul", 1)

    
        right_pipette.pick_up_tip(cuvette_for_transfer[cuvette_tip_wells[i]])
        right_pipette.drop_tip(water_bath[cuvette_tip_wells[i]])
        time.sleep(10)
        protocol.comment("Waiting 10 seconds to simulate heating time.")
    

        #move tubes to delivery area
        right_pipette.pick_up_tip(water_bath[cuvette_tip_wells[i]])
        right_pipette.drop_tip(cuvette_for_transfer[cuvette_tip_wells[i]])
        #take control image of tubes for analysis
        """

