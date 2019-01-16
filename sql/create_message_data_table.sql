-- Table: public.bag_message_data

-- DROP TABLE public.bag_message_data;

CREATE SEQUENCE messageidsequence;

CREATE TABLE public.bag_message_data
(
    messageid bigint DEFAULT NEXTVAL('messageidsequence'),
    positiontime timestamp without time zone,
    bagid bigint NOT NULL,
    typeid character varying(32) COLLATE pg_catalog."default" NOT NULL,
    messagedata jsonb,
    messagetopic character varying COLLATE pg_catalog."default" NOT NULL,
    positionmessageid bigint,
    "position" geometry,
    CONSTRAINT bag_message_pkey PRIMARY KEY (id),
    CONSTRAINT fk_bag_id FOREIGN KEY (bagid)
        REFERENCES public.bags (id) MATCH SIMPLE
        ON UPDATE NO ACTION
        ON DELETE CASCADE,
    CONSTRAINT fk_message_id FOREIGN KEY (typeid)
        REFERENCES public.message_types (md5sum) MATCH SIMPLE
        ON UPDATE NO ACTION
        ON DELETE CASCADE



)
WITH (
    OIDS = FALSE
)
TABLESPACE pg_default;

ALTER TABLE public.bag_message_data
    OWNER to bag_database;

-- Index: bag_message_data_expr_idx

-- DROP INDEX public.bag_message_data_expr_idx;

CREATE INDEX bag_message_data_expr_idx
    ON public.bag_message_data USING btree
    (((((messagedata -> 'twist'::text) -> 'twist'::text) -> 'linear'::text) ->> 'x'::text) COLLATE pg_catalog."default")
    TABLESPACE pg_default;
