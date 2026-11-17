"""Insert two red visual ground markers in the running default Gazebo world."""
import asyncio
import json
from pathlib import Path
from camera_sensor import gz_command

async def main():
    path = Path(__file__).resolve().with_name('red_targets.sdf')
    # JSON quoting also escapes this Protobuf string, including paths with spaces.
    request = 'sdf_filename: ' + json.dumps(str(path)) + ' allow_renaming: false'
    response = await gz_command('service', '-s', '/world/default/create',
        '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
        '--timeout', '5000', '--req', request)
    if 'data: true' not in response.lower():
        raise RuntimeError('Gazebo did not confirm insertion. The targets may already exist.\n'+response)
    print('Two red target tiles added. This did not send any flight commands.')

if __name__ == '__main__':
    asyncio.run(main())
