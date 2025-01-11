
from custom_components.xiaomi_cloud_map_extractor.ijai.map_data_parser  import MapDataParserIjai
from miio.miot_device import MiotDevice

from custom_components.xiaomi_cloud_map_extractor.common.vacuum_v2 import XiaomiCloudVacuumV2
from custom_components.xiaomi_cloud_map_extractor.types import Colors, Drawables, ImageConfig, Sizes, Texts
from custom_components.xiaomi_cloud_map_extractor.common.map_data import MapData

import logging
_LOGGER = logging.getLogger(__name__)

class IjaiVacuum(XiaomiCloudVacuumV2):
    WIFI_STR_LEN = 18
    WIFI_STR_POS = 11

    def __init__(self, connector, country, user_id, device_id, model, token, host, mac):
        self._connector = connector
        self._country = country
        self._user_id = user_id
        self._token = token
        self._host = host
        self._mac = mac
        self._device_id = device_id
        self.model = model
        self._wifi_info_sn = None

    @property
    def map_archive_extension(self) -> str:
        return "zlib"

    @property
    def map_data_parser(self) -> MapDataParserIjai:
        return self._ijai_map_data_parser

    def get_map_url(self, map_name: str) -> str | None:
        url = self._connector.get_api_url(self._country) + '/v2/home/get_interim_file_url_pro'
        params = {
            "data": f'{{"obj_name":"{self._user_id}/{self._device_id}/{map_name}"}}'
        }
        api_response = self._connector.execute_api_call_encrypted(url, params)
        if api_response is None or ("result" not in api_response) or (api_response["result"] is None) or ("url" not in api_response["result"]):
            self._LOGGER.debug(f"API returned {api_response['code']}" + "(" + api_response["message"] + ")")
            return None
        return api_response["result"]["url"]

    def decode_map(self, 
                   raw_map: bytes,
                   colors: Colors,
                   drawables: Drawables,
                   texts: Texts,
                   sizes: Sizes,
                   image_config: ImageConfig) -> MapData:
        GET_PROP_RETRIES=5
        if self._wifi_info_sn is None or self._wifi_info_sn == "":
            _LOGGER.debug(f"host={self._host}, token={self._token}")
            device = MiotDevice(self._host, self._token)
            for _ in range(GET_PROP_RETRIES):
                try:
                    props = device.get_property_by(7, 45)[0]["value"].split(',')
                    self._wifi_info_sn = props[self.WIFI_STR_POS].replace('"', '')[:self.WIFI_STR_LEN]
                    _LOGGER.debug(f"wifi_sn = {self._wifi_info_sn}")
                    break
                except:
                    _LOGGER.warn("Failed to get wifi_sn from vacuum")

        decoded_map = MapDataParserIjai.unpack_map(
            raw_map,
            wifi_sn=self._wifi_info_sn,
            owner_id=str(self._user_id),
            device_id=str(self._device_id),
            model=self.model,
            device_mac=self._mac)
        return MapDataParserIjai.parse(decoded_map, colors, drawables, texts, sizes, image_config)
