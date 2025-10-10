from opentrons import protocol_api

metadata = {
    "protocolName": "Macro Cuvette Test",
    "description": """This protocol is a test for macro cuvettes, based on the opentrons serial dilution tutorial.""",
    "author": "Emil Stabil"
}

requirements = {"robotType": "OT-2", "apiLevel": "2.16"}

def run(protocol: protocol_api.ProtocolContext):
    tips = protocol.load_labware("opentrons_96_tiprack_300ul", 1)
    reservoir = protocol.load_labware("nest_1_reservoir_290ml", 2)
    cuvette_rack = protocol.load_labware("aau_24_tuberack_3000ul", 3)
    right_pipette = protocol.load_instrument("p300_single_gen2", "right", tip_racks=[tips])

    right_pipette.transfer(100, reservoir["A1"], cuvette_rack.wells())

    for i in range(4):
        row = cuvette_rack.rows()[i]
        right_pipette.transfer(100, reservoir["A1"], row[0], mix_after=(3, 50))
        #right_pipette.transfer(100, row[:5], row[1:], mix_after=(3, 50))