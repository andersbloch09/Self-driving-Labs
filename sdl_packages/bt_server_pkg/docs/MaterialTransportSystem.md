# Material Transport System - Design Documentation

## Overview
This system implements intelligent multi-trip material transport using a behavior tree architecture. It handles scenarios where the transport vehicle (MiR robot with 3 slots) has limited capacity compared to the number of materials that need to be transported.

### Container Structure
**Important**: The database is the ground source of truth for material tracking. Materials are stored in the `contents` JSONB field of containers:

```sql
-- Example container entry
{
  "name": "Cuvette_rack_1",
  "contents": {
    "FehlingsSolution": {"amount": 100, "unit": "ml"},
    "GlucoseSolution": {"amount": 50, "unit": "ml"}
  },
  "current_storage_object": "storage_jig_A"
}
```

**Key Points**:
- **Material names** (like "FehlingsSolution") are keys in the `contents` JSONB field
- **Container names** (like "Cuvette_rack_1") are just labels for physical containers
- The system finds containers by searching for materials in the `contents` field
- A single container can hold multiple materials

## Architecture

### Key Design Principles
1. **Database as Source of Truth**: All material tracking and slot management lives in the database
2. **JSONB Contents Queries**: Materials are found by querying the `contents` JSONB field using PostgreSQL operators
3. **Stateless BT Nodes**: Nodes don't track state; they query the database each time
4. **Multi-Trip Logic**: Automatically handles multiple trips when materials exceed capacity
5. **Flexible Destination Mapping**: Destinations provided at runtime via action payload

## Components

### Database Helper Functions (database_service_pkg)

#### `getContainersWithMaterials(material_names)`
Finds containers that hold specific materials by querying the `contents` JSONB field.
```cpp
std::vector<std::string> materials = {"FehlingsSolution", "GlucoseSolution"};
auto containers = database_lib::getContainersWithMaterials(materials);
// Returns: ["Cuvette_rack_1", "Cuvette_rack_3"] (containers holding these materials)
// SQL: SELECT name FROM containers WHERE contents @> jsonb_build_object('FehlingsSolution', '{}')
```

#### `getMaterialsNeedingTransport(material_names, destination_map)`
Determines which materials need to be transported by comparing current vs desired locations.
```cpp
std::map<std::string, std::string> dest_map = {
    {"FehlingsSolution", "storage_ot2"},
    {"GlucoseSolution", "storage_ot2"}
};
std::vector<std::string> materials = {"FehlingsSolution", "GlucoseSolution"};
auto pending = database_lib::getMaterialsNeedingTransport(materials, dest_map);
// Returns: ["FehlingsSolution"] (if GlucoseSolution already at storage_ot2)
```

#### `getAvailableSlotsCount(storage_object_name)`
Returns the number of empty slots in a storage object.
```cpp
int available = database_lib::getAvailableSlotsCount("storage_mir");
// Returns: 3 (if MiR has 3 empty slots)
```

### Behavior Tree Nodes

#### `HasPendingMaterialsCondition` (Condition)
**Purpose**: Check if there are materials still needing transport  
**Returns**: SUCCESS if materials pending, FAILURE if all transported  

**Ports**:
- Input: `materials_json` - JSON array of material names
- Input: `source_storage` - Source storage location (default: "storage_jig_A")
- Input: `transport_storage` - Transport vehicle storage (default: "storage_mir")
- Output: `pending_count` - Number of materials still at source

**Example**:
```xml
<HasPendingMaterialsCondition 
    materials_json='["A", "B", "C"]'
    source_storage="storage_jig_A"
    transport_storage="storage_mir"
    pending_count="{pending_count}" />
```

#### `PlanNextBatchAction` (Action)
**Purpose**: Determine which materials to load in the next batch based on available slots  
**Logic**: `batch_size = min(pending_materials, available_slots)`

**Ports**:
- Input: `materials_json` - JSON array of all material names
- Input: `source_storage` - Source location
- Input: `transport_storage` - Transport vehicle
- Output: `batch_json` - JSON array of materials for this batch
- Output: `batch_size` - Number of materials in batch

**Example Scenario**:
- 4 materials pending: ["A", "B", "C", "D"]
- 3 slots available on MiR
- **Result**: batch = ["A", "B", "C"], batch_size = 3

#### `LoadMaterialsAction` (Action)
**Purpose**: Find containers holding the specified materials and transfer them to transport vehicle  

**Ports**:
- Input: `batch_json` - JSON array of material names to load
- Input: `transport_storage` - Transport vehicle (default: "storage_mir")

**Behavior**:
1. Parses `batch_json` to get list of material names (e.g., ["FehlingsSolution", "GlucoseSolution"])
2. Calls `database_lib::getContainersWithMaterials()` to find which containers hold these materials
3. For each container found, calls `database_lib::moveContainerToStorageObjectByName()` to move it to MiR
4. Updates database to reflect containers now on transport vehicle
5. Fails if any material's container cannot be found or loaded

**Example**: 
- Input: `batch_json = ["FehlingsSolution"]`
- System finds "Cuvette_rack_1" contains FehlingsSolution
- Moves "Cuvette_rack_1" to storage_mir

#### `UnloadMaterialsAction` (Action)
**Purpose**: Transfer containers holding specified materials from transport vehicle to destination  

**Ports**:
- Input: `materials_json` - JSON array of material names
- Input: `transport_storage` - Transport vehicle (default: "storage_mir")
- Input: `destination_storage` - Destination location

**Behavior**:
1. Parses `materials_json` to get material names
2. Calls `database_lib::getContainersWithMaterials()` to find containers holding these materials
3. Filters to only containers currently on transport vehicle (checks current_storage_object)
4. Moves filtered containers to destination storage
5. Safe to call even if no matching containers on vehicle (returns SUCCESS)

## Multi-Trip Transport Logic

### Example Scenario
**Setup**:
- MiR has 3 slots
- 5 materials to transport: ["FehlingsSolution", "GlucoseSolution", "IodideSolution", "StarchSolution", "SodiumThiosulfate"]
- These materials are stored in containers at various locations
- Destination: storage_ot2 (OpenTrons)

**Database State Before**:
```
Container: "Cuvette_rack_1" at storage_jig_A
  contents: {"FehlingsSolution": {...}, "GlucoseSolution": {...}}
  
Container: "Cuvette_rack_2" at storage_jig_A  
  contents: {"IodideSolution": {...}}
  
Container: "Cuvette_rack_3" at storage_jig_A
  contents: {"StarchSolution": {...}, "SodiumThiosulfate": {...}}
```

**Execution Flow**:

#### Trip 1:
1. `HasPendingMaterialsCondition`: ✅ SUCCESS (5 materials pending)
2. `PlanNextBatchAction`: Plans ["FehlingsSolution", "GlucoseSolution", "IodideSolution"] (3 slots available)
3. `LoadMaterialsAction`: 
   - Finds "Cuvette_rack_1" (contains FehlingsSolution, GlucoseSolution)
   - Finds "Cuvette_rack_2" (contains IodideSolution)
   - Loads both containers onto MiR (2 containers, occupying 2 slots)
4. `GoToMission`: MiR drives to destination
5. `UnloadMaterialsAction`: Unloads Cuvette_rack_1 and Cuvette_rack_2 at storage_ot2

#### Trip 2:
1. `HasPendingMaterialsCondition`: ✅ SUCCESS (2 materials pending: StarchSolution, SodiumThiosulfate)
2. `GoToMission`: MiR drives back to source
3. `PlanNextBatchAction`: Plans ["StarchSolution", "SodiumThiosulfate"] (3 slots available, 2 materials pending)
4. `LoadMaterialsAction`: 
   - Finds "Cuvette_rack_3" (contains both materials)
   - Loads onto MiR (1 container, 1 slot)
5. `GoToMission`: MiR drives to destination
6. `UnloadMaterialsAction`: Unloads Cuvette_rack_3 at storage_ot2

#### Loop Exit:
7. `HasPendingMaterialsCondition`: ❌ FAILURE (0 pending)
8. `Inverter`: Converts to SUCCESS
9. `RepeatUntilSuccessful`: Exits loop successfully

## Handling Edge Cases

### Case 1: MiR Already Has Unrelated Materials
**Scenario**: MiR has 1 slot occupied with unrelated container, needs to transport 3 materials

**Solution**:
```
Available slots = 3 total - 1 occupied = 2 slots
PlanNextBatchAction will plan batch of 2 materials (not 3)
System automatically handles reduced capacity
```

### Case 2: All Materials Already Transported
**Behavior**: First condition check fails, loop never enters, returns SUCCESS immediately

### Case 3: Multiple Materials in One Container
**Scenario**: Container "Cuvette_rack_1" holds both "FehlingsSolution" and "GlucoseSolution"

**Behavior**: 
- System recognizes the container holds multiple requested materials
- Loads the container once (occupies 1 slot)
- Both materials are delivered together
- More efficient than treating them separately

### Case 4: Material in Multiple Containers
**Scenario**: "WaterSample" exists in both "Container_A" and "Container_B"

**Behavior**:
- `getContainersWithMaterials()` returns both containers
- System will load both (if slots available)
- Destination receives material from multiple sources

### Case 5: Database Connection Failure
**Behavior**: Nodes return FAILURE, tree execution stops with error

## ROS2 Action Interface

### Starting Material Transport
```bash
ros2 action send_goal /bt_execution \
  btcpp_ros2_interfaces/action/ExecuteTree \
  "{target_tree: 'MaterialTransport', \
    payload: '{ \
       \"materials_json\": \"[\\\"FehlingsSolution\\\", \\\"GlucoseSolution\\\", \\\"IodideSolution\\\", \\\"StarchSolution\\\"]\", \
       \"destination_map\": { \
         \"FehlingsSolution\": \"storage_ot2\", \
         \"GlucoseSolution\": \"storage_ot2\", \
         \"IodideSolution\": \"storage_ot2\", \
         \"StarchSolution\": \"storage_ot2\" \
       }, \
       \"mission_map\": { \
         \"storage_jig_A\": \"94c9f0cf-a4f7-11f0-b2e5-000e8e984489\", \
         \"storage_ot2\": \"a1b2c3d4-e5f6-7890-1234-567890abcdef\" \
       } \
    }' \
  }" \
  --feedback
```

### Payload Parameters
- `materials_json`: JSON string array of **material names** (not container names) to transport
  - Example: `["FehlingsSolution", "GlucoseSolution"]`
  - These are keys in the container `contents` JSONB field
- `destination_map`: JSON object mapping each material to its desired storage location
  - Example: `{"FehlingsSolution": "storage_ot2"}`
- `mission_map`: JSON object mapping storage locations to MiR mission UUIDs
  - Example: `{"storage_jig_A": "94c9f0cf-...", "storage_ot2": "a1b2c3d4-..."}`

## Testing

### Unit Test: Database Functions
```bash
# Test available slots count
ros2 run database_service test_available_slots

# Test pending materials calculation
ros2 run database_service test_pending_materials
```

### Integration Test: Behavior Tree
```bash
# Run test tree with 4 materials, 3 slots (expects 2 trips)
ros2 action send_goal /bt_execution \
  btcpp_ros2_interfaces/action/ExecuteTree \
  "{target_tree: 'MaterialTransportTest'}" \
  --feedback
```

## Future Enhancements

### Placeholder: PlaceOpentronsAction
**Purpose**: Place materials into specific OpenTrons deck slots  
**Design**:
```xml
<PlaceOpentronsAction name="place_into_opentrons"
                     material_name="{material_name}"
                     deck_slot="{deck_slot}"
                     action_name="place_opentrons_material" />
```

**Integration Point**:
- Add after `go_to_destination` in MaterialTransport.xml
- Would need per-material destination mapping
- Could iterate through batch_json with ForEach decorator

### Priority Ordering
Add material priority field to database, modify `PlanNextBatchAction` to sort by priority.

### Capacity Prediction
Pre-compute number of trips needed and display to user before starting.

### Collision Avoidance
Integrate with MiR fleet management to coordinate multiple robots.

## Troubleshooting

### "No containers found for material X"
- Check database: Does a container exist with this material in `contents`?
  ```sql
  SELECT name, contents FROM containers 
  WHERE contents ? 'FehlingsSolution';
  ```
- Verify material name matches exactly (case-sensitive)
- Check JSONB structure: material should be a top-level key

### "No pending materials" immediately
- Verify `destination_map` correctly specifies where materials should go
- Check if materials are already at their destination:
  ```sql
  SELECT c.name, c.contents, c.current_storage_object 
  FROM containers c 
  WHERE contents ? 'FehlingsSolution';
  ```
- Ensure material names in `materials_json` match keys in container `contents`

### "No available slots on transport vehicle"
- Query database: 
  ```sql
  SELECT * FROM slots 
  WHERE storage_object_id = (
    SELECT id FROM storage_objects WHERE name = 'storage_mir'
  );
  ```
- Check if slots are occupied by unrelated containers
- Verify slot status is correctly updated

### Materials not moving between trips
- Check database logs: `SELECT * FROM movements ORDER BY timestamp DESC LIMIT 10`
- Verify `moveContainerToStorageObjectByName()` returning true
- Check slot updates in database
- Confirm container names found by `getContainersWithMaterials()` are valid

### Container not found but material exists
- Verify the container with that material is at the expected source location
- Check if container was already moved to a different storage object
- Query: 
  ```sql
  SELECT c.name, c.current_storage_object, c.contents->'MaterialName'
  FROM containers c
  WHERE contents ? 'MaterialName';
  ```

## Database Schema Requirements

### Tables Used
- `containers`: id, name, current_slot_id, current_storage_object, **contents (JSONB)**
  - `contents` structure: `{"MaterialName": {"amount": 100, "unit": "ml"}, ...}`
- `slots`: id, name, storage_object_id, container_id, status
- `storage_objects`: id, name, type
- `movements`: container_id, from_slot, to_slot, moved_by, timestamp

### Example Container Entry
```sql
INSERT INTO containers (name, contents, current_storage_object) VALUES 
('Cuvette_rack_1', 
 '{"FehlingsSolution": {"amount": 100, "unit": "ml"}, 
   "GlucoseSolution": {"amount": 50, "unit": "ml"}}'::jsonb,
 'storage_jig_A');
```

### Required Storage Objects
- Source (e.g., "storage_jig_A")
- Transport vehicle ("storage_mir")
- Destination (e.g., "storage_ot2")
