using System.Collections;
using UnityEngine;

using UnitySensors.DataType.Geometry;
using UnitySensors.Interface.Geometry;

namespace UnitySensors.Sensor.GNSS
{
    public class GNSSSensor : UnitySensor, IGeoCoordinateInterface
    {
        private GeoCoordinateSystem _coordinateSystem;
        private Transform _transform;

        [SerializeField]
        GeoCoordinate _coordinate;
        public GeoCoordinate coordinate { get => _coordinate; }

        protected override void Init()
        {
            _transform = this.transform;
            _coordinateSystem = FindFirstObjectByType<GeoCoordinateSystem>();
        }

        protected override IEnumerator UpdateSensor()
        {
            _coordinate = _coordinateSystem.GetCoordinate(_transform.position);
            yield return null;
        }

        protected override void OnSensorDestroy()
        {
        }
    }
}
