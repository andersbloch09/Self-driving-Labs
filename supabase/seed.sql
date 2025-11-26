--
-- PostgreSQL database dump
--

-- Dumped from database version 15.8
-- Dumped by pg_dump version 15.8

SET statement_timeout = 0;
SET lock_timeout = 0;
SET idle_in_transaction_session_timeout = 0;
SET client_encoding = 'UTF8';
SET standard_conforming_strings = on;
SELECT pg_catalog.set_config('search_path', '', false);
SET check_function_bodies = false;
SET xmloption = content;
SET client_min_messages = warning;
SET row_security = off;

--
-- Name: public; Type: SCHEMA; Schema: -; Owner: pg_database_owner
--

CREATE SCHEMA public;


ALTER SCHEMA public OWNER TO pg_database_owner;

--
-- Name: SCHEMA public; Type: COMMENT; Schema: -; Owner: pg_database_owner
--

COMMENT ON SCHEMA public IS 'standard public schema';


SET default_tablespace = '';

SET default_table_access_method = heap;

--
-- Name: containers; Type: TABLE; Schema: public; Owner: supabase_admin
--

CREATE TABLE public.containers (
    id uuid DEFAULT gen_random_uuid() NOT NULL,
    name text NOT NULL,
    contents jsonb,
    current_slot_id uuid,
    current_storage_object_id uuid DEFAULT gen_random_uuid(),
    in_transit boolean,
    last_update timestamp with time zone,
    notes text,
    slots jsonb
);


ALTER TABLE public.containers OWNER TO supabase_admin;

--
-- Name: movements; Type: TABLE; Schema: public; Owner: supabase_admin
--

CREATE TABLE public.movements (
    id uuid DEFAULT gen_random_uuid() NOT NULL,
    container_id uuid NOT NULL,
    from_slot uuid,
    to_slot uuid DEFAULT gen_random_uuid(),
    "timestamp" timestamp with time zone DEFAULT now(),
    moved_by text
);


ALTER TABLE public.movements OWNER TO supabase_admin;

--
-- Name: TABLE movements; Type: COMMENT; Schema: public; Owner: supabase_admin
--

COMMENT ON TABLE public.movements IS 'Logging of movements';


--
-- Name: slots; Type: TABLE; Schema: public; Owner: supabase_admin
--

CREATE TABLE public.slots (
    id uuid DEFAULT gen_random_uuid() NOT NULL,
    storage_object_id uuid NOT NULL,
    name text,
    transform_to_object jsonb,
    status text DEFAULT '''available''::text'::text,
    container_id uuid
);


ALTER TABLE public.slots OWNER TO supabase_admin;

--
-- Name: TABLE slots; Type: COMMENT; Schema: public; Owner: supabase_admin
--

COMMENT ON TABLE public.slots IS 'Slots in Storage Objects';


--
-- Name: storage_objects; Type: TABLE; Schema: public; Owner: supabase_admin
--

CREATE TABLE public.storage_objects (
    id uuid DEFAULT gen_random_uuid() NOT NULL,
    name text NOT NULL,
    location text,
    parent_id uuid,
    transform_to_parent jsonb,
    description text
);


ALTER TABLE public.storage_objects OWNER TO supabase_admin;

--
-- Name: TABLE storage_objects; Type: COMMENT; Schema: public; Owner: supabase_admin
--

COMMENT ON TABLE public.storage_objects IS 'Storage Objects';


--
-- Data for Name: containers; Type: TABLE DATA; Schema: public; Owner: supabase_admin
--

INSERT INTO public.containers (id, name, contents, current_slot_id, current_storage_object_id, in_transit, last_update, notes, slots) VALUES ('69625f14-8e7b-40c5-80e8-9c9c1355787e', 'Cuvette_rack_1', '{}', NULL, NULL, NULL, '2025-10-31 08:54:09+00', NULL, '{"A1": [1, 0, 3], "A2": [1, 4]}');
INSERT INTO public.containers (id, name, contents, current_slot_id, current_storage_object_id, in_transit, last_update, notes, slots) VALUES ('b411fb2a-1f6b-4f8e-a99b-063bff998519', 'Container1', '{"chemical": "FehlingsA"}', 'd0d65c6f-5855-4082-b168-04711f09ba28', NULL, NULL, '2025-10-30 09:46:34+00', NULL, NULL);


--
-- Data for Name: movements; Type: TABLE DATA; Schema: public; Owner: supabase_admin
--

INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('45fc6606-cda2-4c70-87fc-362a2307993b', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', NULL, '08b0667f-c45c-48d7-98ef-fa64e1ef44d1', '2025-11-13 13:37:00.505697+00', 'Blochen');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('55c48701-c8dd-438f-956e-4363d1b1f685', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '08b0667f-c45c-48d7-98ef-fa64e1ef44d1', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-18 12:16:32.071646+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('73039d0b-9b79-4161-9d5b-ddee81f1a69b', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '9264b323-a5c6-40e4-828e-4db2dece9146', '2025-11-18 12:18:32.9697+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('233c07fe-ce38-4011-aa89-008c652d8ab0', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '9264b323-a5c6-40e4-828e-4db2dece9146', '08b0667f-c45c-48d7-98ef-fa64e1ef44d1', '2025-11-18 12:19:28.502234+00', 'Andes');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('bc1b7dd0-a398-4727-8cea-8146ae4bbc3f', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '08b0667f-c45c-48d7-98ef-fa64e1ef44d1', '77e232f8-c719-4e63-bfd0-14bd24d03f5c', '2025-11-18 12:19:49.116965+00', 'blochen');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('d20f0341-cf40-4407-bec2-4c18c73d91e7', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '77e232f8-c719-4e63-bfd0-14bd24d03f5c', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-18 12:20:30.759082+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('d1288781-7b13-41a9-adfb-93ea07d8e564', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '5a8e44e6-9fcd-49e1-a0c9-45190ff86e4e', '2025-11-18 12:22:30.030882+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('34033e75-2cf6-4edc-bc6e-9da0cfe59a85', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5a8e44e6-9fcd-49e1-a0c9-45190ff86e4e', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-18 12:25:35.497815+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('20f60cc8-3dcd-4cbf-837a-12f3386050c0', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '5a8e44e6-9fcd-49e1-a0c9-45190ff86e4e', '2025-11-18 12:28:22.924049+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('26dcc199-6d6a-418f-a48b-221c92a951e4', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5a8e44e6-9fcd-49e1-a0c9-45190ff86e4e', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-18 12:29:50.650428+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('a3d14219-6042-4b04-b260-1c641e0d3d66', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '65b5560b-17b4-4cce-a5e2-2f4bb3e202bb', '2025-11-18 12:33:14.731764+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('73490cbe-38bc-45e3-8f16-217d56da7061', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '65b5560b-17b4-4cce-a5e2-2f4bb3e202bb', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-18 12:37:08.628307+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('35b11a1f-da81-4d4e-a824-7acb2f4b0094', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '08b0667f-c45c-48d7-98ef-fa64e1ef44d1', '2025-11-18 13:59:14.307714+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('8f76cb78-e2b0-43eb-9b3c-4e228ab295f7', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '08b0667f-c45c-48d7-98ef-fa64e1ef44d1', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-18 14:00:54.278654+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('16f26d0c-b010-4d09-a37b-d7ca1aa0933a', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '08b0667f-c45c-48d7-98ef-fa64e1ef44d1', '2025-11-19 12:00:16.883701+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('cfdf5d63-4165-4842-94d8-5104acee1f7e', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '08b0667f-c45c-48d7-98ef-fa64e1ef44d1', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-19 12:01:06.686338+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('022004f0-9952-4fd5-800c-ad8917d7c617', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '08b0667f-c45c-48d7-98ef-fa64e1ef44d1', '2025-11-19 12:02:20.30639+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('a3a7a430-efd8-4975-b00f-f281f4e7ad93', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '08b0667f-c45c-48d7-98ef-fa64e1ef44d1', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-19 12:03:07.284019+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('2ebee2c5-08ca-4648-899a-c2d35cf9b29a', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '65b5560b-17b4-4cce-a5e2-2f4bb3e202bb', '2025-11-20 08:47:41.14119+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('bab0cb0f-f63c-4f60-a317-e46d30fa086b', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '65b5560b-17b4-4cce-a5e2-2f4bb3e202bb', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-20 08:52:35.022919+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('c1babbad-3fab-4239-8188-c3716092c243', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '9264b323-a5c6-40e4-828e-4db2dece9146', '2025-11-20 08:53:35.763883+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('11f3e427-987f-4813-a917-b55a20ca24f1', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '9264b323-a5c6-40e4-828e-4db2dece9146', '65b5560b-17b4-4cce-a5e2-2f4bb3e202bb', '2025-11-20 08:56:31.876863+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('008399d0-572f-40f7-a20d-e6f57db15699', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '65b5560b-17b4-4cce-a5e2-2f4bb3e202bb', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-20 08:58:14.305726+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('908755b8-7f75-4867-9581-c07fa270aa70', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '9264b323-a5c6-40e4-828e-4db2dece9146', '2025-11-20 08:59:41.107225+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('43a7f622-56ba-47b6-bd01-63ac30b6ae6d', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '9264b323-a5c6-40e4-828e-4db2dece9146', '08b0667f-c45c-48d7-98ef-fa64e1ef44d1', '2025-11-20 09:07:25.355237+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('990d79b8-dd4e-4c09-b20b-6f964039f5dc', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '08b0667f-c45c-48d7-98ef-fa64e1ef44d1', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-20 09:08:09.847989+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('47d35367-6def-4b29-abef-5d58fe068e99', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '08b0667f-c45c-48d7-98ef-fa64e1ef44d1', '2025-11-20 09:08:45.620337+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('e6831e13-35c7-4e58-9169-e1d402d256f5', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '08b0667f-c45c-48d7-98ef-fa64e1ef44d1', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-20 09:09:18.918761+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('5a801c54-a104-43a1-ab33-bcffcb0741d4', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '08b0667f-c45c-48d7-98ef-fa64e1ef44d1', '2025-11-20 09:09:53.162549+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('f081feec-cc5f-4711-9d08-3dbb6d6b71ff', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '08b0667f-c45c-48d7-98ef-fa64e1ef44d1', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-20 09:10:27.790192+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('ddbe99d4-3273-4e41-8e35-e4e2e8c2be61', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '08b0667f-c45c-48d7-98ef-fa64e1ef44d1', '2025-11-20 09:11:01.452629+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('739f76f5-7190-4d33-b26b-7e88c7b2e672', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '08b0667f-c45c-48d7-98ef-fa64e1ef44d1', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 09:16:45.652872+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('bc01d8de-ddbe-4d6a-8921-010344f11cd0', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 09:27:07.408749+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('979c98ae-a1e4-49df-9519-795377a9320b', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 09:39:09.620633+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('ab4eb415-484b-4a5f-9059-72efd0783edc', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 09:39:48.8451+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('65e0ca39-f153-4566-a8b2-8bcd807a87f3', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '9264b323-a5c6-40e4-828e-4db2dece9146', '2025-11-21 09:45:18.85176+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('5acf6aab-99e9-4fae-b0bf-c22c3afdc4a2', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '9264b323-a5c6-40e4-828e-4db2dece9146', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 10:30:58.248907+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('2991dae6-e778-42d1-b377-aa15b0625430', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 10:32:19.753156+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('df19129f-5de0-4819-a78f-8fb87be543ea', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 10:34:49.668392+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('e390d954-e9ec-4ed7-a0a7-b2c72fcd5e36', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 10:35:42.363758+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('8ed7e2a2-9c5c-4429-a555-1d9b7ec2089e', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 10:35:55.239821+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('00b8454e-c4ba-4650-8fa6-f3f75d5af6c5', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 10:37:32.325755+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('7a6f2a6d-ec0a-4fb7-b762-dac4c0275636', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '9264b323-a5c6-40e4-828e-4db2dece9146', '2025-11-21 10:43:34.567146+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('67e796f9-fbf0-4bf8-afe7-0ddac6cc5e9b', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '9264b323-a5c6-40e4-828e-4db2dece9146', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 10:46:39.721539+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('0752ebdf-d37a-4b60-b21a-361bd8a98fae', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 10:47:48.72491+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('18418851-08dd-4b72-b806-023b321da32b', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 10:48:40.350015+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('2b8f0ebd-1dc7-4aea-8086-b11aa74e2863', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 10:48:41.451012+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('eef231b1-0f57-48f9-82c7-bffe5afab396', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '9264b323-a5c6-40e4-828e-4db2dece9146', '2025-11-21 10:49:45.908014+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('1770b705-ca3f-4fb8-9e5c-f671d98f586b', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '9264b323-a5c6-40e4-828e-4db2dece9146', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 10:50:47.642281+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('eb22c920-5ce4-4293-97aa-a341bf644ff6', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 10:51:51.423621+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('4f7d9168-8eef-45b6-805b-829ca13fb03b', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 10:52:36.999251+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('14f3dbf6-f437-4a93-a67f-a56c8d0d915e', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 10:55:10.606369+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('86780d8d-c92f-45e1-a285-6bdd958bca9b', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 10:55:53.338695+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('37032f82-35f9-4ee9-bad7-10dc41811533', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 10:57:35.44155+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('a89f4bc0-c32c-479c-adac-334edd65bfc0', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '9264b323-a5c6-40e4-828e-4db2dece9146', '2025-11-21 11:11:28.364201+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('4d73f702-4057-4984-81a5-3e92c8f60504', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '9264b323-a5c6-40e4-828e-4db2dece9146', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 11:23:18.468026+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('c2de6006-e57f-4864-bc3f-643b24196290', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 11:25:38.552904+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('09932b29-7ac5-4172-bd89-135cd956bbfd', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 11:25:46.472159+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('d08542f5-e6bf-446c-aa61-dd91c626f893', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 11:28:27.411763+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('13d3a848-42e1-4224-99c5-e7c262b6196b', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 11:32:21.844964+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('1e75e3a9-a761-4d4b-b39c-0a30e7746b98', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 11:32:28.498462+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('c8cba7fc-34f9-4ec1-bee6-306442b49818', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 11:35:35.667335+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('aea017c8-9b91-463f-9391-4ce06ee3687a', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 11:38:22.20222+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('88bb4b7a-e050-419b-9440-0dec1b35a878', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 11:40:17.325845+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('82623eef-f17b-4718-aae1-5c93cd01bab4', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 11:40:26.175493+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('fc8db70d-72b3-4146-89ca-8f58f3793a4c', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 11:43:27.75991+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('99f2ad2c-215a-4f5a-8bcb-ec31ce2bf2e7', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 11:45:51.482588+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('df30657d-5302-4e20-af0b-1a13984cf039', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 11:46:57.662682+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('c775b734-c407-46bd-9110-63ea34fa19ab', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 11:48:31.056188+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('45f8b426-64c6-46aa-8977-140b942c3a25', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 11:53:21.577568+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('2508adf8-b1bf-4608-ac89-82ae28ce46da', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 11:54:50.60507+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('94f27330-47c9-436a-a357-bbd4c3a6f6ff', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 11:59:31.199108+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('455a1494-a780-4c28-9455-8d5d181f94cf', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 12:02:24.864337+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('20e7a658-5c16-4293-874b-75741fae30a2', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 12:03:46.702837+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('b20781e4-3b78-449c-92cc-b1376b48d919', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 12:06:25.86341+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('0b404834-ef60-485a-8845-4a917d409b28', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 12:08:59.474488+00', 'a');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('7f93a721-a06c-44fc-a09d-0cf2d1b5d2b1', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 12:09:43.434522+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('59d8804a-a007-4abb-b1ed-cb02bba5df95', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 12:09:43.619936+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('98608285-1b80-49d1-8888-4f7df0fa24a7', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 12:10:42.192724+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('0ebaee3c-7ab8-4078-82cf-f10ed3bde8b1', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 12:11:48.499855+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('40a8afa6-9b23-41a9-8183-0a17274d74bf', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 12:15:06.475923+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('2038c755-e4c8-4c76-bcf7-243b0a5c7300', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 12:15:41.532071+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('b98c5cff-7e2b-40fd-a809-1497f8379510', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 12:18:20.148756+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('d77c17d0-e871-47c5-96d0-a955bcb0d27c', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 12:19:34.56254+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('7c504214-2575-47ce-a094-203330cff62a', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 12:20:37.653467+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('064a5cfd-d413-42d2-beb8-77156fdb78b4', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 12:21:50.069577+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('48434c3d-fa58-4e64-b8a2-37fd0646014f', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 12:24:15.525342+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('9194ec4f-773f-47d6-988a-4c4f5c5e2b40', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 12:26:10.492403+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('be21edfc-1906-44d4-be2e-211f4654eb7b', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 12:29:59.272696+00', 'a');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('1f773cd7-8907-4ce0-9a4f-5f7ca351b844', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 12:34:00.342652+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('af9e6599-2592-4516-a986-a8043ba2dcf0', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 12:36:31.08456+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('9baedeef-89d1-49d2-90df-321db55ad00d', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 12:40:23.505987+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('35119637-68ec-4886-92bf-9293e30891f1', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 12:40:59.585914+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('8d997c58-be69-4d4f-8d9e-11eb55bffd34', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 12:41:47.412954+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('026b20cf-3322-4de2-be8b-2b4ddbeb41a3', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 12:42:48.170629+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('a8a5d9e7-65ea-4a35-9a98-9475ad004efd', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 12:45:55.168548+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('182c0b95-32a6-4a2e-bf55-f556740c5864', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 12:46:34.557147+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('044dfcbd-89a1-4866-a4be-953638f6dc2f', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 12:49:32.727194+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('21f62935-aba7-48e6-a815-c4d874fc445d', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 12:50:48.762875+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('8c0b318c-8659-4113-90a9-2c4eb90e5db7', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 12:52:54.4807+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('115a7b38-cd90-4694-a9a6-0d3e2b1d5893', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 12:56:42.844649+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('d182b6c3-21db-4244-9053-baa315d80eca', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 12:59:06.490353+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('c0c3daab-3768-45dd-9894-a051d8760252', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 13:02:00.668383+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('0c5413a6-2c2f-4904-a6ea-48db0717a63f', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 13:05:03.770077+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('b619e01f-6cae-4304-a153-1a3edd400268', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 13:07:12.712225+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('4e61d477-2073-4771-8e4e-83377e22874b', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 13:08:46.756682+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('c123aaa4-52ed-424b-9370-052adfb82170', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 13:11:33.888952+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('da2da210-36ac-4edf-bd6e-516c6f94681f', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 13:12:18.273701+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('0cca0672-9786-46c8-acbc-628766862635', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 13:13:55.025304+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('d3ceb883-0974-4fc0-bfc7-da6882d03d87', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 13:15:09.982331+00', 'a');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('e42a9082-d334-48b7-828d-c8e5a6f2d436', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 13:16:32.900125+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('1ea1b232-8f16-43ad-b3c4-593c1e0dac96', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '2025-11-21 13:17:10.099182+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('4b52f047-9a19-4452-b83f-c069295e4280', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 13:25:59.033523+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('3910cc70-c0ae-4d55-bb81-d58ce18e88c1', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '2025-11-21 13:26:28.631744+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('4c4636b0-f5d1-4a13-aa96-cdfccac780dd', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 13:31:58.983558+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('a92099e1-e95f-4da5-9c3e-48f30cef323f', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '2025-11-21 13:32:43.688237+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('a66e4f31-73ab-4627-970c-6be219094d8e', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 13:32:50.955413+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('42383874-2a6e-4171-961a-fed716b3196a', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 13:33:30.830238+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('c4f3e438-08ff-42e2-b677-3e314aaa7636', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 13:35:40.431106+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('3b2ea900-7668-43c4-b3cc-1749335e2c62', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 13:43:11.032851+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('c680e5a9-45bc-47fc-8af5-bf2cea85f700', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 14:21:32.768663+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('72cce313-53c2-41c6-8c5e-299035537a10', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 14:29:24.790004+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('7ffb3e23-3d1f-4e7f-a064-ffb66b6a9b71', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 14:30:53.567914+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('0fcd9cca-97a6-4b45-9797-78e5bf775f23', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '9264b323-a5c6-40e4-828e-4db2dece9146', '2025-11-21 14:37:01.081732+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('6ddd4969-f1e8-41cd-b75e-fa62509c5792', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '9264b323-a5c6-40e4-828e-4db2dece9146', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 14:37:37.418292+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('9bb50b4f-8c18-4747-9f0d-02680a655bef', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 14:39:40.615112+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('17ed77d4-a7c6-42dd-bf3b-aece95e7a9bc', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 14:42:33.73225+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('7918710f-00b9-4695-a1d2-ce8eda7ed9de', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 14:43:46.707786+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('de2dc2b5-1ca4-48b5-8778-5fd8ff6a8720', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 14:44:30.49587+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('10a29923-6a43-4335-944e-f1a535564211', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 14:44:31.285641+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('bd483a5b-3a0d-4654-9f74-43d8b43944a3', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 14:47:46.349174+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('5708eb95-538b-4113-9543-061fccb8f07f', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 14:48:52.279518+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('9c2bfda5-45c3-4b86-ba6e-d5e02e08935d', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 14:51:14.350205+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('3ed5ab13-3a8b-4c6a-bb6c-71740337f181', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 14:55:23.479395+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('2b117613-8dd9-4dcb-8a72-02b7d935f4b8', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 14:57:12.155209+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('de123323-5735-4003-b1c3-9d3aa08b5b66', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 14:57:54.900838+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('5ad36ca5-939b-45cd-b69a-e73405292f3b', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 14:58:19.302643+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('bf4b295a-15da-485f-8684-9e1203ab630d', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 15:01:20.797393+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('9f979698-4cb1-4458-9d7b-450427014bc7', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 15:03:00.005454+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('ad586ec7-b987-4c84-944d-1a09597bcea0', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 15:04:51.833798+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('7471c6f0-ec12-41fd-8aaf-6ab6b603cabb', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 15:05:29.875193+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('5855de98-d467-4411-ae99-285baf932cdb', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 15:06:53.503222+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('59c9f407-1847-4a6f-bc02-19e1202840c5', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 15:14:41.749388+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('391ea54c-c40d-46b4-832a-afe7c03dd4c2', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 15:15:57.070891+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('e059e81e-f6ed-40be-a87b-bb33d8d2b581', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 15:17:06.971116+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('e47fbfe7-2210-4d51-bddb-5a66594bf2dd', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 15:18:35.709208+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('9d5e65a5-5944-4f57-81ac-feb377885fd6', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 15:20:08.594175+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('4404dd2b-6d6b-40b5-922f-d607a4e38f86', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 15:20:15.767324+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('d03fd00c-5e79-4646-822d-4369cfd72e5b', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 15:21:53.816926+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('883ebb91-038f-4746-a419-8b0ba3ad82db', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 15:21:58.283169+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('baacb56d-67ad-42f4-af9a-83c05799fe02', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 15:22:06.040443+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('d8ad940c-4479-4a81-8d6a-75cba1bc830a', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 15:24:04.540481+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('326fd0e7-6f73-4eaf-ab4f-044b3021da38', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 15:45:42.414881+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('0d910a62-9aac-494d-8331-a05dd3303438', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 15:47:28.117284+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('c0aa6823-a087-4fc6-bc70-faf63fa6f377', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 15:49:46.537739+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('67e1845d-4f37-4f3f-88e0-3f9369bb20cf', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 15:50:04.773085+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('516ee044-3984-4e1d-9a57-562b87c04a17', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 15:51:15.590096+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('29f0adef-0b4e-4d2b-8dcf-3202f6dba18a', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 15:51:27.98724+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('4f175b72-0924-4a52-9b8e-100ff01968b1', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 15:52:17.121015+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('53373757-52a3-4b0f-a25d-fe32cef9d690', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 15:52:50.413859+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('3ec475d8-c8d9-4d4c-898c-fc40673a2ae4', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 15:53:58.006906+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('3a5e09c0-dea5-4104-ba4f-6fa90a34fb31', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 15:56:54.088863+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('9df96153-6e2e-43e7-8e8d-98cab1656aeb', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 15:59:30.619242+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('3d063a8b-01d2-4516-a057-3a0b1656baad', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 15:59:55.707114+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('1e87e2eb-e206-47c9-a34a-66ed91f6f685', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 16:00:32.263161+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('0c984a9c-998c-40ac-b0bb-02a8d40de9cd', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 16:01:31.192512+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('635dac46-72ae-4b3a-8823-85caefa5ff05', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 16:05:56.50012+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('6e9e5506-62d9-4961-b615-1f734664c802', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 16:07:01.647319+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('6d486e24-7d23-4dee-b1ed-d83e5e5087db', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 16:08:42.339464+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('1378757e-6ddc-4d26-81ae-5a4a62742077', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 16:09:19.438187+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('9f34ace4-ca62-462f-a4de-e9b9c4ea5190', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 16:09:42.600238+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('9e9e413a-dc09-4ad2-bf0d-bd49730a2279', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 16:11:16.948051+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('c2592264-b857-4d0e-aa93-94f7eecd3520', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 16:13:13.693387+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('7b6934e9-6c18-445a-be56-cb06d1a1f351', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 16:15:03.461081+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('71a376bd-410d-4645-851c-8d2e8bdfbf42', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 16:15:25.705747+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('6c7a3c70-72e7-4599-9c05-f91e17faeaaf', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 16:16:32.476428+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('35b48809-bbcc-4802-9d0b-4e5da3b1cd07', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-21 16:16:50.307271+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('645ebab0-9006-4a23-8e25-286ffad23b41', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 16:19:02.043631+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('60124b3b-d5f8-494e-96e2-0b6e4abfae10', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 16:19:08.533323+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('cc769a9a-b387-4e6b-8fa8-864704cfb220', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 16:20:01.269172+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('10ec1244-3d64-476a-a8a0-6ae3ccffd3b2', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 16:20:23.262617+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('25b22da3-851f-4eda-8b8f-f24a39dc1d93', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 16:21:06.862496+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('d9a2c91d-2833-41b5-a054-7c27e94f6a0c', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 16:21:51.921997+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('cf9f2f4d-8954-4d5d-a258-40557b48e650', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 16:22:42.910598+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('eff83a35-be84-4edd-be61-83b44b81f60a', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 16:23:18.224767+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('83bec440-ba67-4c42-b169-9c7d15555f23', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 16:24:08.445058+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('13192609-ec89-4ce3-95fa-ce54d1332d9e', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 16:25:34.461612+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('487b89b9-64f9-4907-a5c3-83aef12cd519', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 16:25:39.416771+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('f9ba5d13-9360-4be4-8348-3a5fa51423af', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 16:26:52.43358+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('0b87e64e-7a53-4048-bc7c-eba893551007', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 16:28:19.599503+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('b93c9ca5-5bdd-4468-89f7-14febc7436a6', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 16:30:52.211492+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('2a3b564b-7065-4341-9160-66bcce84f6a5', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 16:34:22.601175+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('e12a5edc-1030-4212-978b-5f63f200709d', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 16:35:19.005333+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('4dbc75d7-8ded-4539-9114-ed8c95312304', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 16:36:22.610983+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('22b9775e-dd99-4983-928a-411f1d34113c', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 16:39:36.796793+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('2f1360ea-1537-4ca8-bc9e-5b835f1523ef', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 16:40:32.606323+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('d132fe63-fa75-464f-a09e-c135c099aef8', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 16:41:15.541655+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('427f0db4-537a-4c79-80a9-3ae1c648ba69', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 16:43:26.910056+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('f2b23c13-69fe-4499-836c-166a175731df', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 16:45:40.526397+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('09ccafb0-9499-4a5a-95fc-314d40b64b00', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 16:46:58.539094+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('3ea5b9a7-c311-4fd5-8e08-cacd51ac1054', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 16:47:59.694713+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('9cf1dc74-ac44-4788-8bf5-b3353785846d', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 16:48:54.829603+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('63fb627c-2a93-4f11-a8c1-8eef33613fd7', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '2025-11-21 16:48:59.877027+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('e7341a93-3e3e-4189-afeb-6be0c3e2b3d4', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 16:49:48.57483+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('2be5edd9-f067-431a-81e0-da01a4522092', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '2025-11-21 16:50:32.056465+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('1cf3fc36-36a8-440a-b402-b7f3fd812cbc', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 16:51:10.508001+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('0ec25dd6-6526-4044-bcbf-5de1fdfad044', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '2025-11-21 16:51:21.638941+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('9a668e3c-6dee-4eeb-91f1-30249ea8f8ff', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 16:52:14.424661+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('4abc1682-b4db-46d1-841b-f1a292ac5cb1', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '2025-11-21 16:52:45.873355+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('586343f2-28f3-403c-8ef9-b45e245e3e85', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '2025-11-21 16:54:46.802898+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('949d2440-cc02-4a50-a34a-4c7b3413a184', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 16:55:55.96354+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('2d1537ca-fa6a-4997-a9f3-8f0719c111ff', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 16:56:55.631696+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('599613d6-9b10-4d30-8d04-8be31a82657b', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 16:58:31.420123+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('38feaa5d-005f-4374-b1e1-824b5ccc1e71', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 16:58:39.542801+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('3d91c3f5-146a-422a-aea2-8fcc50cf0c93', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 17:00:00.953519+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('962876b1-b3aa-4bd2-8ad9-fb52e67ce3d3', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 17:02:28.589199+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('e23efc43-f522-43d2-98d6-ea7d7cbaa809', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 17:03:56.212667+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('6e534fde-e497-4fde-8163-9d071725e420', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 17:04:09.405988+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('2d92c9c6-c9fa-4052-9024-0021f0f011b0', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 17:05:12.08498+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('6f6e7f79-e3a4-4f5e-823c-4b6b07f01756', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 17:06:38.176904+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('718f255d-367a-446c-9ef6-c870ab53d3b4', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 17:10:41.900041+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('633f8586-f93b-4285-b36f-60c82cebe4ff', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 17:19:20.412342+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('c18d2e43-035d-4bb6-8df1-3db432bf65c6', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 17:20:15.167012+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('ee48511a-b081-4208-b768-682c3e7f55ad', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-21 17:22:18.42779+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('2ce99634-e2c4-412f-a16f-8d6003269621', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 17:26:38.944492+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('1ed9c5f5-434e-4ae1-82d7-b334326ffee1', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 17:32:09.321824+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('ad59d572-c7bb-4dd3-8a5b-7d5cec394c08', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-21 17:34:07.74255+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('12dd7aed-30ac-46fa-ae07-f119d089da94', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 09:05:25.874753+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('a801d821-9e2d-4e47-acd6-ab2d289077ea', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-25 09:05:55.749944+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('d097f2d0-1552-4d18-b555-ea6aeeed7217', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 09:06:39.658869+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('93ff4cbc-5175-48c2-89c5-a24c376fb305', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '2025-11-25 09:10:20.643748+00', 'aa');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('a55cd433-1a46-4585-aa11-4e3392f2b10e', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 09:11:20.172896+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('4fe529cb-6b73-42ca-82ff-cdf90eceeeda', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '2025-11-25 09:16:33.744475+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('8d6ac0e3-8c62-491e-9c1b-443f7b7fe442', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '2025-11-25 09:59:19.214149+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('9f72ce07-26ec-4191-9be3-e18c38f4ea0d', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 10:00:10.613935+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('5b2c4cf4-4659-4cfe-bd03-b8cc423d7837', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '2025-11-25 10:00:24.697923+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('8d432687-41a5-4501-8187-a7754e5a3fe3', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 10:01:16.807756+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('5a2ecaad-2296-4391-9cb3-82d71033aed5', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '2025-11-25 10:01:28.097998+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('8302cbe8-f1f0-4466-b94c-f31a3e05cfd5', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 10:03:23.758613+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('fb1aafdb-d0c0-4728-ab7f-a229e04deda1', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '2025-11-25 10:03:34.179221+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('40d2efb4-f55b-4bad-b008-87afbac8f881', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 10:06:07.852655+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('5babb5c3-b141-4a07-b98a-d2c1f7572d0a', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '2025-11-25 10:07:22.795959+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('f7bc52fd-4aab-4fb7-9c22-2942af8a191a', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '2025-11-25 10:10:16.244457+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('dba0f677-125e-4ffb-af2f-5f5a6769bcae', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 10:16:32.505763+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('8859c033-9538-4061-8154-f7a9c5b9319c', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-25 10:16:53.111101+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('79c1edf9-859e-4865-8928-cc29ec2237a5', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 10:17:30.210736+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('d3dde48c-0b56-496f-89e7-3ac89add180c', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-25 11:07:35.26177+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('d1c80c68-4857-4356-8d37-79931c0c9f0c', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 11:09:24.825376+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('04e9f200-c221-4224-9205-a5081909432c', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-25 11:09:36.549075+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('83b9f10b-5304-4a02-999c-f0165a857b9b', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 11:11:01.681864+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('df8e9fb5-29d9-4586-9548-261f0076773f', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-25 11:11:33.89945+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('e8090a79-ec2b-46cb-bdfb-ba762ae0185c', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 11:14:22.907994+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('3d3c80f8-39d0-4060-b716-6803c18ed8af', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-25 12:09:16.255148+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('7d310ded-bf72-41ec-978e-4800a30d9428', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 12:10:48.519891+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('694b8b84-523d-4c9c-81f6-083ff9c7999a', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '2025-11-25 12:11:25.792943+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('11f25631-fefb-4b4b-8869-65aa738af40d', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 12:12:21.603793+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('513b8bce-f555-4ead-b8a3-eae88ee144db', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '2025-11-25 12:13:16.625061+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('1974f8b6-a342-4c71-a71c-a88b027a25d5', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 12:14:33.738096+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('2eff472e-0098-4ee4-a7ff-5dedc047ab55', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '2025-11-25 12:20:04.96678+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('6bd7183e-f177-4c3c-8ce7-56f875f53857', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 12:21:14.157888+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('2b0e3a3f-420b-4bd8-8cd2-93f81b699a94', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '2025-11-25 12:21:31.915355+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('7c330613-b0cd-4058-9c8c-9bca0c20d4db', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 12:23:32.289578+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('09fd0171-e7d0-4e12-95a1-90b4fbd80891', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '2025-11-25 12:26:09.172274+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('0a15fb4e-d7dd-40f2-acda-b2d596424af0', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 12:27:14.754641+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('c3d26826-fa0b-4994-b91c-2abfa3b21d5a', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '2025-11-25 12:27:29.905259+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('efc7a6a7-661b-41c7-9d37-68440a250bdf', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 12:28:27.533999+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('aa13ccfb-9242-46b1-a55a-8adf9cb356d0', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '2025-11-25 12:28:34.053455+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('ba76a6c9-4516-4c16-a5ff-0c7aad7184a6', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '2025-11-25 12:38:22.912391+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('8cc4fd3c-1eba-415d-934c-7404f2de0738', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 12:39:32.698274+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('7ad1e948-5f69-46ff-8960-538366ab0de6', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '2025-11-25 12:45:46.902798+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('8aa26618-ce6b-400a-81fb-b09455fc99c1', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 12:46:49.452128+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('929e49bf-1548-491d-a36d-97b871829616', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '2025-11-25 12:51:46.688169+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('519c5fdc-eab4-41ae-a0cd-ec1401e38e87', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-25 12:52:41.974508+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('65d8b784-e65d-4c9e-a09a-0fd67698a186', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 12:52:42.966224+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('6600e37c-c347-41e6-bbd0-037d9cea85bb', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-25 12:52:53.132439+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('e195c10e-5620-4e80-8863-a03a397be6ac', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-25 12:53:51.500333+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('288cfe18-1235-4a4b-bda7-9b2a3a9b3bdd', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 12:55:41.982795+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('d590a13f-e6be-4425-84d5-f6f6458bd2d7', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-25 12:57:09.765672+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('03334958-85cf-48b1-9e61-ed37c261219c', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 12:58:36.048874+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('89113739-cbd1-47a1-824b-3b60a2b8413a', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-25 13:00:30.130549+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('8828b65c-e51c-4b5f-aac3-54bc20a3c853', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 13:01:38.8741+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('954af2a8-f899-4ea1-8feb-ae27fc87abc9', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-25 13:02:26.30779+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('e96d3363-c94f-4321-8090-48ad276023bf', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-25 13:08:18.174595+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('e42b18fb-6c53-402e-9481-2ebb7c5a657c', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 13:11:30.923431+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('5f2a62b4-beb0-4ce5-b9a7-f8a13769852c', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-25 13:11:49.516804+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('c30bf28d-5a21-40ae-82f2-222bd4759f0b', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 13:12:36.771176+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('7dea0194-8853-4db6-add2-7cd3e33fabd2', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', '2025-11-25 13:13:23.494257+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('cb24d771-35c4-4c3a-8c0a-90045719519e', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '5d8d460f-02f2-4504-9f80-9b74d64f30fc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 13:14:41.911313+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('a818b259-964e-4482-bdc3-1b7fd30ea62d', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-25 13:14:58.514025+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('9118ad4c-b202-4247-84b2-67013cab3ab9', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 13:15:54.31323+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('14d4770c-e12f-4c3f-b5bd-ab17222b2eda', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-25 13:16:54.252158+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('383a8b39-bef3-4155-bad0-1731c8eb27d5', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 13:20:03.839006+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('a52db232-e317-48b8-9cff-a407a25b82df', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-25 13:21:16.326286+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('8c32b36b-76e4-4082-a7d8-2e243a4ce4ae', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 13:28:41.341824+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('05021a61-1b97-470c-bae7-2b2668ef8d55', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-25 13:32:00.131962+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('bb3aadac-f67c-4023-b802-b45ace59fb86', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 13:32:50.851263+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('a7547168-51fa-4514-a844-9e9ff1dc2d11', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-25 13:33:01.814052+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('d4fbfbc2-4082-4632-8d9e-6c37d3942ea1', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-25 13:35:30.588389+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('3bc2b504-3548-4f14-8a3d-72fcbd2b4525', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 13:37:32.089896+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('b346e116-a9fc-4fdc-a3a2-815ed5cda572', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-25 13:39:34.995795+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('3a416590-8249-4eed-b9d9-440703f99db6', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 13:42:26.599995+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('4d85453e-0152-4fa6-a6e7-b56a8cba4e1d', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '89552f9a-3682-4a12-ba60-90d6b46a0238', '2025-11-25 13:44:48.528628+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('8bc33553-ecd1-4ae7-8b24-80aeeec5cdbf', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '89552f9a-3682-4a12-ba60-90d6b46a0238', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 13:46:09.783247+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('4026af77-8717-4bc6-ba16-5effcd31ce63', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-25 13:46:36.177671+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('58871507-13bb-40c6-83b7-79e95a6b2c97', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 13:48:15.881419+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('a80ce3f2-af7b-4713-8634-f1d01c958d5a', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-25 13:48:46.043209+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('d668d67a-1165-4453-adc7-db3dc7f76792', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-25 13:50:53.915283+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('eda231d0-77cd-4fa4-a525-61a04536d34a', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-11-25 13:52:45.176213+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('56cdb4d9-2c92-4e39-97ad-6ceb2b496f1e', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 14:30:59.045538+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('a4096f36-df7b-47f5-b16e-4bd154356680', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 14:41:11.302466+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('39ead296-f7b8-480f-b130-72592df0f6ed', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-25 14:49:53.482766+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('09a73f26-f45f-48d5-a6fb-b1fb5aa0411f', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-26 07:58:07.422164+00', 'robot_system');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('be7bcd8b-56c1-4962-8b29-58fcfb3ce388', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-11-26 09:27:06.559049+00', 'robot_system');


--
-- Data for Name: slots; Type: TABLE DATA; Schema: public; Owner: supabase_admin
--

INSERT INTO public.slots (id, storage_object_id, name, transform_to_object, status, container_id) VALUES ('77e232f8-c719-4e63-bfd0-14bd24d03f5c', 'b41ad3ad-162b-4801-ab11-557cf26b2883', 'B2', '{"slot": "B2", "translation": [0.285, -0.12, 0], "rotation_RPY": [0, 0, 0]}', 'available', NULL);
INSERT INTO public.slots (id, storage_object_id, name, transform_to_object, status, container_id) VALUES ('81212c3f-74a0-4885-a5f6-47ad34ecb809', '3d1b4309-ab56-4437-bda5-77fdc9b6f3f8', 'M3', '{"slot": "M3", "translation": [0, 0.12, 0], "rotation_RPY": [0, 0, 0]}', 'available', NULL);
INSERT INTO public.slots (id, storage_object_id, name, transform_to_object, status, container_id) VALUES ('d0d65c6f-5855-4082-b168-04711f09ba28', '3d1b4309-ab56-4437-bda5-77fdc9b6f3f8', 'M1', '{"slot": "M1", "translation": [0, -0.12, 0], "rotation_RPY": [0, 0, 0]}', 'occupied', 'b411fb2a-1f6b-4f8e-a99b-063bff998519');
INSERT INTO public.slots (id, storage_object_id, name, transform_to_object, status, container_id) VALUES ('89552f9a-3682-4a12-ba60-90d6b46a0238', '5e7b1d78-788e-49a6-8e46-ef8f70ee1648', '2', '{"slot": "2", "translation": [0.15, -0.2383, 0.18], "rotation_RPY": [89, 0, -89]}', 'available', NULL);
INSERT INTO public.slots (id, storage_object_id, name, transform_to_object, status, container_id) VALUES ('08b0667f-c45c-48d7-98ef-fa64e1ef44d1', 'b41ad3ad-162b-4801-ab11-557cf26b2883', 'A1', '{"slot": "A1", "translation": [0.125, 0, 0], "rotation_RPY": [0, 0, 0]}', 'available', NULL);
INSERT INTO public.slots (id, storage_object_id, name, transform_to_object, status, container_id) VALUES ('9264b323-a5c6-40e4-828e-4db2dece9146', '3d1b4309-ab56-4437-bda5-77fdc9b6f3f8', 'M2', '{"slot": "M2", "translation": [0, 0, 0], "rotation_RPY": [0, 0, 0]}', 'available', NULL);
INSERT INTO public.slots (id, storage_object_id, name, transform_to_object, status, container_id) VALUES ('7df969fb-9f96-479f-acdb-3aacd70724bc', '5e7b1d78-788e-49a6-8e46-ef8f70ee1648', '1', '{"slot": "1", "translation": [0.15, -0.37, 0.18], "rotation_RPY": [89, 0, -89]}', 'available', NULL);
INSERT INTO public.slots (id, storage_object_id, name, transform_to_object, status, container_id) VALUES ('5a8e44e6-9fcd-49e1-a0c9-45190ff86e4e', 'b41ad3ad-162b-4801-ab11-557cf26b2883', 'A2', '{"slot": "A2", "translation": [0.125, -0.12, 0], "rotation_RPY": [0, 0, 0]}', 'available', NULL);
INSERT INTO public.slots (id, storage_object_id, name, transform_to_object, status, container_id) VALUES ('5d8d460f-02f2-4504-9f80-9b74d64f30fc', '5e7b1d78-788e-49a6-8e46-ef8f70ee1648', '3', '{"slot": "3", "translation": [0.15, -0.1058, 0.18], "rotation_RPY": [89, 0, -89]}', 'available', NULL);
INSERT INTO public.slots (id, storage_object_id, name, transform_to_object, status, container_id) VALUES ('65b5560b-17b4-4cce-a5e2-2f4bb3e202bb', 'b41ad3ad-162b-4801-ab11-557cf26b2883', 'B1', '{"slot": "B1", "translation": [0.285, 0, 0], "rotation_RPY": [0, 0, 0]}', 'available', NULL);


--
-- Data for Name: storage_objects; Type: TABLE DATA; Schema: public; Owner: supabase_admin
--

INSERT INTO public.storage_objects (id, name, location, parent_id, transform_to_parent, description) VALUES ('b41ad3ad-162b-4801-ab11-557cf26b2883', 'storage_jig_A', 'storage', NULL, '{"translation": [0, 0, 0], "rotation_quat": [0, 0, 0, 1]}', 'Storage Jig for Containers');
INSERT INTO public.storage_objects (id, name, location, parent_id, transform_to_parent, description) VALUES ('5e7b1d78-788e-49a6-8e46-ef8f70ee1648', 'storage_ot2', 'opentrons', NULL, '{"translation": [0, 0, 0], "rotation_quat": [0, 0, 0, 1]}', 'The available slots in the Opentrons workspace');
INSERT INTO public.storage_objects (id, name, location, parent_id, transform_to_parent, description) VALUES ('3d1b4309-ab56-4437-bda5-77fdc9b6f3f8', 'storage_mir', 'robot_base', NULL, '{"translation": [0, 0, 0], "rotation_quat": [0, 0, 0, 1]}', 'The storage available on the MiR_Base');


--
-- Name: containers containers_name_key; Type: CONSTRAINT; Schema: public; Owner: supabase_admin
--

ALTER TABLE ONLY public.containers
    ADD CONSTRAINT containers_name_key UNIQUE (name);


--
-- Name: containers containers_pkey; Type: CONSTRAINT; Schema: public; Owner: supabase_admin
--

ALTER TABLE ONLY public.containers
    ADD CONSTRAINT containers_pkey PRIMARY KEY (id);


--
-- Name: movements movements_pkey; Type: CONSTRAINT; Schema: public; Owner: supabase_admin
--

ALTER TABLE ONLY public.movements
    ADD CONSTRAINT movements_pkey PRIMARY KEY (id);


--
-- Name: slots slots_pkey; Type: CONSTRAINT; Schema: public; Owner: supabase_admin
--

ALTER TABLE ONLY public.slots
    ADD CONSTRAINT slots_pkey PRIMARY KEY (id);


--
-- Name: storage_objects storage_objects_name_key; Type: CONSTRAINT; Schema: public; Owner: supabase_admin
--

ALTER TABLE ONLY public.storage_objects
    ADD CONSTRAINT storage_objects_name_key UNIQUE (name);


--
-- Name: storage_objects storage_objects_pkey; Type: CONSTRAINT; Schema: public; Owner: supabase_admin
--

ALTER TABLE ONLY public.storage_objects
    ADD CONSTRAINT storage_objects_pkey PRIMARY KEY (id);


--
-- Name: containers containers_current_slot_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: supabase_admin
--

ALTER TABLE ONLY public.containers
    ADD CONSTRAINT containers_current_slot_id_fkey FOREIGN KEY (current_slot_id) REFERENCES public.slots(id) ON DELETE SET NULL;


--
-- Name: containers containers_current_storage_object_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: supabase_admin
--

ALTER TABLE ONLY public.containers
    ADD CONSTRAINT containers_current_storage_object_id_fkey FOREIGN KEY (current_storage_object_id) REFERENCES public.storage_objects(id) ON DELETE SET NULL;


--
-- Name: movements movements_container_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: supabase_admin
--

ALTER TABLE ONLY public.movements
    ADD CONSTRAINT movements_container_id_fkey FOREIGN KEY (container_id) REFERENCES public.containers(id);


--
-- Name: movements movements_from_slot_fkey; Type: FK CONSTRAINT; Schema: public; Owner: supabase_admin
--

ALTER TABLE ONLY public.movements
    ADD CONSTRAINT movements_from_slot_fkey FOREIGN KEY (from_slot) REFERENCES public.slots(id);


--
-- Name: movements movements_to_slot_fkey; Type: FK CONSTRAINT; Schema: public; Owner: supabase_admin
--

ALTER TABLE ONLY public.movements
    ADD CONSTRAINT movements_to_slot_fkey FOREIGN KEY (to_slot) REFERENCES public.slots(id);


--
-- Name: slots slots_container_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: supabase_admin
--

ALTER TABLE ONLY public.slots
    ADD CONSTRAINT slots_container_id_fkey FOREIGN KEY (container_id) REFERENCES public.containers(id) ON DELETE SET NULL;


--
-- Name: slots slots_storage_object_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: supabase_admin
--

ALTER TABLE ONLY public.slots
    ADD CONSTRAINT slots_storage_object_id_fkey FOREIGN KEY (storage_object_id) REFERENCES public.storage_objects(id) ON DELETE CASCADE;


--
-- Name: storage_objects storage_objects_parent_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: supabase_admin
--

ALTER TABLE ONLY public.storage_objects
    ADD CONSTRAINT storage_objects_parent_id_fkey FOREIGN KEY (parent_id) REFERENCES public.storage_objects(id) ON DELETE CASCADE;


--
-- Name: containers; Type: ROW SECURITY; Schema: public; Owner: supabase_admin
--

ALTER TABLE public.containers ENABLE ROW LEVEL SECURITY;

--
-- Name: movements; Type: ROW SECURITY; Schema: public; Owner: supabase_admin
--

ALTER TABLE public.movements ENABLE ROW LEVEL SECURITY;

--
-- Name: slots; Type: ROW SECURITY; Schema: public; Owner: supabase_admin
--

ALTER TABLE public.slots ENABLE ROW LEVEL SECURITY;

--
-- Name: storage_objects; Type: ROW SECURITY; Schema: public; Owner: supabase_admin
--

ALTER TABLE public.storage_objects ENABLE ROW LEVEL SECURITY;

--
-- Name: SCHEMA public; Type: ACL; Schema: -; Owner: pg_database_owner
--

GRANT USAGE ON SCHEMA public TO postgres;
GRANT USAGE ON SCHEMA public TO anon;
GRANT USAGE ON SCHEMA public TO authenticated;
GRANT USAGE ON SCHEMA public TO service_role;


--
-- Name: TABLE containers; Type: ACL; Schema: public; Owner: supabase_admin
--

GRANT ALL ON TABLE public.containers TO postgres;
GRANT ALL ON TABLE public.containers TO anon;
GRANT ALL ON TABLE public.containers TO authenticated;
GRANT ALL ON TABLE public.containers TO service_role;


--
-- Name: TABLE movements; Type: ACL; Schema: public; Owner: supabase_admin
--

GRANT ALL ON TABLE public.movements TO postgres;
GRANT ALL ON TABLE public.movements TO anon;
GRANT ALL ON TABLE public.movements TO authenticated;
GRANT ALL ON TABLE public.movements TO service_role;


--
-- Name: TABLE slots; Type: ACL; Schema: public; Owner: supabase_admin
--

GRANT ALL ON TABLE public.slots TO postgres;
GRANT ALL ON TABLE public.slots TO anon;
GRANT ALL ON TABLE public.slots TO authenticated;
GRANT ALL ON TABLE public.slots TO service_role;


--
-- Name: TABLE storage_objects; Type: ACL; Schema: public; Owner: supabase_admin
--

GRANT ALL ON TABLE public.storage_objects TO postgres;
GRANT ALL ON TABLE public.storage_objects TO anon;
GRANT ALL ON TABLE public.storage_objects TO authenticated;
GRANT ALL ON TABLE public.storage_objects TO service_role;


--
-- Name: DEFAULT PRIVILEGES FOR SEQUENCES; Type: DEFAULT ACL; Schema: public; Owner: postgres
--

ALTER DEFAULT PRIVILEGES FOR ROLE postgres IN SCHEMA public GRANT ALL ON SEQUENCES  TO postgres;
ALTER DEFAULT PRIVILEGES FOR ROLE postgres IN SCHEMA public GRANT ALL ON SEQUENCES  TO anon;
ALTER DEFAULT PRIVILEGES FOR ROLE postgres IN SCHEMA public GRANT ALL ON SEQUENCES  TO authenticated;
ALTER DEFAULT PRIVILEGES FOR ROLE postgres IN SCHEMA public GRANT ALL ON SEQUENCES  TO service_role;


--
-- Name: DEFAULT PRIVILEGES FOR SEQUENCES; Type: DEFAULT ACL; Schema: public; Owner: supabase_admin
--

ALTER DEFAULT PRIVILEGES FOR ROLE supabase_admin IN SCHEMA public GRANT ALL ON SEQUENCES  TO postgres;
ALTER DEFAULT PRIVILEGES FOR ROLE supabase_admin IN SCHEMA public GRANT ALL ON SEQUENCES  TO anon;
ALTER DEFAULT PRIVILEGES FOR ROLE supabase_admin IN SCHEMA public GRANT ALL ON SEQUENCES  TO authenticated;
ALTER DEFAULT PRIVILEGES FOR ROLE supabase_admin IN SCHEMA public GRANT ALL ON SEQUENCES  TO service_role;


--
-- Name: DEFAULT PRIVILEGES FOR FUNCTIONS; Type: DEFAULT ACL; Schema: public; Owner: postgres
--

ALTER DEFAULT PRIVILEGES FOR ROLE postgres IN SCHEMA public GRANT ALL ON FUNCTIONS  TO postgres;
ALTER DEFAULT PRIVILEGES FOR ROLE postgres IN SCHEMA public GRANT ALL ON FUNCTIONS  TO anon;
ALTER DEFAULT PRIVILEGES FOR ROLE postgres IN SCHEMA public GRANT ALL ON FUNCTIONS  TO authenticated;
ALTER DEFAULT PRIVILEGES FOR ROLE postgres IN SCHEMA public GRANT ALL ON FUNCTIONS  TO service_role;


--
-- Name: DEFAULT PRIVILEGES FOR FUNCTIONS; Type: DEFAULT ACL; Schema: public; Owner: supabase_admin
--

ALTER DEFAULT PRIVILEGES FOR ROLE supabase_admin IN SCHEMA public GRANT ALL ON FUNCTIONS  TO postgres;
ALTER DEFAULT PRIVILEGES FOR ROLE supabase_admin IN SCHEMA public GRANT ALL ON FUNCTIONS  TO anon;
ALTER DEFAULT PRIVILEGES FOR ROLE supabase_admin IN SCHEMA public GRANT ALL ON FUNCTIONS  TO authenticated;
ALTER DEFAULT PRIVILEGES FOR ROLE supabase_admin IN SCHEMA public GRANT ALL ON FUNCTIONS  TO service_role;


--
-- Name: DEFAULT PRIVILEGES FOR TABLES; Type: DEFAULT ACL; Schema: public; Owner: postgres
--

ALTER DEFAULT PRIVILEGES FOR ROLE postgres IN SCHEMA public GRANT ALL ON TABLES  TO postgres;
ALTER DEFAULT PRIVILEGES FOR ROLE postgres IN SCHEMA public GRANT ALL ON TABLES  TO anon;
ALTER DEFAULT PRIVILEGES FOR ROLE postgres IN SCHEMA public GRANT ALL ON TABLES  TO authenticated;
ALTER DEFAULT PRIVILEGES FOR ROLE postgres IN SCHEMA public GRANT ALL ON TABLES  TO service_role;


--
-- Name: DEFAULT PRIVILEGES FOR TABLES; Type: DEFAULT ACL; Schema: public; Owner: supabase_admin
--

ALTER DEFAULT PRIVILEGES FOR ROLE supabase_admin IN SCHEMA public GRANT ALL ON TABLES  TO postgres;
ALTER DEFAULT PRIVILEGES FOR ROLE supabase_admin IN SCHEMA public GRANT ALL ON TABLES  TO anon;
ALTER DEFAULT PRIVILEGES FOR ROLE supabase_admin IN SCHEMA public GRANT ALL ON TABLES  TO authenticated;
ALTER DEFAULT PRIVILEGES FOR ROLE supabase_admin IN SCHEMA public GRANT ALL ON TABLES  TO service_role;


--
-- PostgreSQL database dump complete
--

