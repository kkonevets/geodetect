begin; 

create or replace function geo.tg_geometry_notify ()
  returns trigger
  language plpgsql
as $$
  declare
  channel text := TG_ARGV[0];
begin
  PERFORM (
    with payload(id, tablename, op) as
	(
	  select
            (case when NEW is NULL THEN OLD.id ELSE NEW.id END) as id,
	    TG_TABLE_NAME,
	    TG_OP
	)
    select pg_notify(channel, payload::text)
      from payload
  );
  RETURN NULL;
end;
$$;


DROP TRIGGER IF EXISTS geometry_modify
  ON geo.gz_polygon;    
DROP TRIGGER IF EXISTS geometry_modify
  ON geo.gz_line;    
DROP TRIGGER IF EXISTS geometry_modify
  ON geo.gz_circle;    

CREATE TRIGGER geometry_modify
  AFTER INSERT OR UPDATE OR DELETE
  ON geo.gz_polygon
  FOR EACH ROW
    EXECUTE PROCEDURE geo.tg_geometry_notify('geometry_modify');

CREATE TRIGGER geometry_modify
  AFTER INSERT OR UPDATE OR DELETE
  ON geo.gz_line
  FOR EACH ROW
    EXECUTE PROCEDURE geo.tg_geometry_notify('geometry_modify');

CREATE TRIGGER geometry_modify
  AFTER INSERT OR UPDATE OR DELETE
  ON geo.gz_circle
  FOR EACH ROW
    EXECUTE PROCEDURE geo.tg_geometry_notify('geometry_modify');

commit;

--listen "geometry_modify";
--insert into geo.gz_polygon(gz_id, polygon) values (1, ST_AsBinary(ST_GeomFromText('POLYGON((0 0, 1 0, 1 1, 0 1, 0 0))')));
--delete from geo.gz_polygon where id=7625;
