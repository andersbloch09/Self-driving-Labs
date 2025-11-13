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

INSERT INTO public.containers (id, name, contents, current_slot_id, current_storage_object_id, in_transit, last_update, notes, slots) VALUES ('69625f14-8e7b-40c5-80e8-9c9c1355787e', 'Cuvette_rack_1', '{}', 'd0d65c6f-5855-4082-b168-04711f09ba28', '3d1b4309-ab56-4437-bda5-77fdc9b6f3f8', NULL, '2025-10-31 08:54:09+00', NULL, '{"A1": [1, 0, 3], "A2": [1, 4]}');
INSERT INTO public.containers (id, name, contents, current_slot_id, current_storage_object_id, in_transit, last_update, notes, slots) VALUES ('b411fb2a-1f6b-4f8e-a99b-063bff998519', 'Container1', '{"chemical": "FehlingsA"}', '08b0667f-c45c-48d7-98ef-fa64e1ef44d1', 'b41ad3ad-162b-4801-ab11-557cf26b2883', NULL, '2025-10-30 09:46:34+00', NULL, NULL);


--
-- Data for Name: movements; Type: TABLE DATA; Schema: public; Owner: supabase_admin
--

INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('17702099-039a-4ae7-8055-32463faf3845', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '08b0667f-c45c-48d7-98ef-fa64e1ef44d1', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-10-31 12:06:19.364929+00', 'TEST');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('788a1954-04d4-406c-8721-56f2f6bc644e', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '9264b323-a5c6-40e4-828e-4db2dece9146', '2025-10-31 12:22:38.450509+00', 'TEST');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('ef959ee1-d88d-491c-992f-fb22c2057e9f', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '9264b323-a5c6-40e4-828e-4db2dece9146', '9264b323-a5c6-40e4-828e-4db2dece9146', '2025-10-31 12:32:27.919889+00', 'TEST');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('063462a4-ebdd-4a30-a49f-8450e499d968', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '9264b323-a5c6-40e4-828e-4db2dece9146', '08b0667f-c45c-48d7-98ef-fa64e1ef44d1', '2025-10-31 12:32:50.28418+00', 'auto_placement');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('ee1643b7-4973-4987-8b0f-bf2a31773c87', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '08b0667f-c45c-48d7-98ef-fa64e1ef44d1', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-10-31 12:40:46.108726+00', 'auto_placement');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('22aa9c82-25aa-4574-aa60-d95e30a9eee0', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '7df969fb-9f96-479f-acdb-3aacd70724bc', '2025-10-31 12:48:09.000961+00', 'TEST');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('73de7398-1723-4f16-87ba-0597779046f2', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', '7df969fb-9f96-479f-acdb-3aacd70724bc', 'd0d65c6f-5855-4082-b168-04711f09ba28', '2025-10-31 13:07:52.42209+00', 'TEST');
INSERT INTO public.movements (id, container_id, from_slot, to_slot, "timestamp", moved_by) VALUES ('31c88b51-c2c7-43f2-96ee-04b590f33e0d', 'b411fb2a-1f6b-4f8e-a99b-063bff998519', 'd0d65c6f-5855-4082-b168-04711f09ba28', '08b0667f-c45c-48d7-98ef-fa64e1ef44d1', '2025-10-31 13:08:16.30745+00', 'auto_placement');


--
-- Data for Name: slots; Type: TABLE DATA; Schema: public; Owner: supabase_admin
--

INSERT INTO public.slots (id, storage_object_id, name, transform_to_object, status, container_id) VALUES ('89552f9a-3682-4a12-ba60-90d6b46a0238', '5e7b1d78-788e-49a6-8e46-ef8f70ee1648', '2', '{"translation": [0, 0, 0], "rotation_quat": [0, 0, 0, 1]}', 'available', NULL);
INSERT INTO public.slots (id, storage_object_id, name, transform_to_object, status, container_id) VALUES ('5d8d460f-02f2-4504-9f80-9b74d64f30fc', '5e7b1d78-788e-49a6-8e46-ef8f70ee1648', '3', '{"translation": [0, 0, 0], "rotation_quat": [0, 0, 0, 1]}', 'available', NULL);
INSERT INTO public.slots (id, storage_object_id, name, transform_to_object, status, container_id) VALUES ('ee92eff4-0529-49ff-8f01-e6a6710fbda4', '5e7b1d78-788e-49a6-8e46-ef8f70ee1648', '4', '{"translation": [0, 0, 0], "rotation_quat": [0, 0, 0, 1]}', 'available', NULL);
INSERT INTO public.slots (id, storage_object_id, name, transform_to_object, status, container_id) VALUES ('9264b323-a5c6-40e4-828e-4db2dece9146', '3d1b4309-ab56-4437-bda5-77fdc9b6f3f8', 'M2', '{"translation": [0, 0, 0], "rotation_quat": [0, 0, 0, 1]}', 'available', NULL);
INSERT INTO public.slots (id, storage_object_id, name, transform_to_object, status, container_id) VALUES ('5a8e44e6-9fcd-49e1-a0c9-45190ff86e4e', 'b41ad3ad-162b-4801-ab11-557cf26b2883', 'A2', '{"translation": [28.5, 0, 0], "rotation_quat": [0, 0, 0, 1]}', 'available', NULL);
INSERT INTO public.slots (id, storage_object_id, name, transform_to_object, status, container_id) VALUES ('d09a652c-f4f4-4eba-a82f-aa2e4c0537b4', '5e7b1d78-788e-49a6-8e46-ef8f70ee1648', '8', '{"translation": [0, 0, 0], "rotation_quat": [0, 0, 0, 1]}', 'available', NULL);
INSERT INTO public.slots (id, storage_object_id, name, transform_to_object, status, container_id) VALUES ('4b41b49f-ed92-4bc6-b089-8b2ebf4e364a', '5e7b1d78-788e-49a6-8e46-ef8f70ee1648', '10', '{"translation": [0, 0, 0], "rotation_quat": [0, 0, 0, 1]}', 'available', NULL);
INSERT INTO public.slots (id, storage_object_id, name, transform_to_object, status, container_id) VALUES ('bd2717f0-7036-4058-8951-6cbe6cdad3ac', '5e7b1d78-788e-49a6-8e46-ef8f70ee1648', '11', '{"translation": [0, 0, 0], "rotation_quat": [0, 0, 0, 1]}', 'available', NULL);
INSERT INTO public.slots (id, storage_object_id, name, transform_to_object, status, container_id) VALUES ('7062ce39-2439-4d06-b5a7-12a704d0b14c', '5e7b1d78-788e-49a6-8e46-ef8f70ee1648', '5', '{"translation": [0, 0, 0], "rotation_quat": [0, 0, 0, 1]}', 'available', NULL);
INSERT INTO public.slots (id, storage_object_id, name, transform_to_object, status, container_id) VALUES ('0272410a-c132-47f5-9ee1-c63c0cfc2bae', '5e7b1d78-788e-49a6-8e46-ef8f70ee1648', '6', '{"translation": [0, 0, 0], "rotation_quat": [0, 0, 0, 1]}', 'available', NULL);
INSERT INTO public.slots (id, storage_object_id, name, transform_to_object, status, container_id) VALUES ('386b3007-dde1-45f0-acda-3a222ac84636', '5e7b1d78-788e-49a6-8e46-ef8f70ee1648', '9', '{"translation": [0, 0, 0], "rotation_quat": [0, 0, 0, 1]}', 'available', NULL);
INSERT INTO public.slots (id, storage_object_id, name, transform_to_object, status, container_id) VALUES ('519d2f7a-e749-4b19-84bf-5bfdff0c4662', '5e7b1d78-788e-49a6-8e46-ef8f70ee1648', '7', '{"translation": [0, 0, 0], "rotation_quat": [0, 0, 0, 1]}', 'available', NULL);
INSERT INTO public.slots (id, storage_object_id, name, transform_to_object, status, container_id) VALUES ('7df969fb-9f96-479f-acdb-3aacd70724bc', '5e7b1d78-788e-49a6-8e46-ef8f70ee1648', '1', '{"translation": [0, 0, 0], "rotation_quat": [0, 0, 0, 1]}', 'available', NULL);
INSERT INTO public.slots (id, storage_object_id, name, transform_to_object, status, container_id) VALUES ('65b5560b-17b4-4cce-a5e2-2f4bb3e202bb', 'b41ad3ad-162b-4801-ab11-557cf26b2883', 'B1', '{"translation": [12.5, -12, 0], "rotation_quat": [0, 0, 0, 1]}', 'available', NULL);
INSERT INTO public.slots (id, storage_object_id, name, transform_to_object, status, container_id) VALUES ('77e232f8-c719-4e63-bfd0-14bd24d03f5c', 'b41ad3ad-162b-4801-ab11-557cf26b2883', 'B2', '{"translation": [28.5, -12, 0], "rotation_quat": [0, 0, 0, 1]}', 'available', NULL);
INSERT INTO public.slots (id, storage_object_id, name, transform_to_object, status, container_id) VALUES ('d0d65c6f-5855-4082-b168-04711f09ba28', '3d1b4309-ab56-4437-bda5-77fdc9b6f3f8', 'M1', '{"translation": [0, 12, 0], "rotation_quat": [0, 0, 0, 1]}', 'available', NULL);
INSERT INTO public.slots (id, storage_object_id, name, transform_to_object, status, container_id) VALUES ('81212c3f-74a0-4885-a5f6-47ad34ecb809', '3d1b4309-ab56-4437-bda5-77fdc9b6f3f8', 'M3', '{"translation": [0, -12, 0], "rotation_quat": [0, 0, 0, 1]}', 'available', NULL);
INSERT INTO public.slots (id, storage_object_id, name, transform_to_object, status, container_id) VALUES ('08b0667f-c45c-48d7-98ef-fa64e1ef44d1', 'b41ad3ad-162b-4801-ab11-557cf26b2883', 'A1', '{"translation": [12.5, 0, 0], "rotation_quat": [0, 0, 0, 1]}', 'available', 'b411fb2a-1f6b-4f8e-a99b-063bff998519');


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

