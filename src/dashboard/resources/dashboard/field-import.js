/* Local-only field image importer. No HTTP, NT publication, or actuator API. */
export const FIELD_MAP_VERSION = 'frc-field-map/1';
const MAX_BYTES = 8 * 1024 * 1024, MAX_PIXELS = 16_000_000, MAX_DIM = 8192;
const EPS = 1e-10;
const copy = value => JSON.parse(JSON.stringify(value));
const finite = value => typeof value === 'number' && Number.isFinite(value);
const eq = (a, b) => Math.abs(a - b) <= EPS;
const same = (a, b) => Array.isArray(a) && Array.isArray(b) && eq(a[0], b[0]) && eq(a[1], b[1]);
const area = ring => ring.slice(0, -1).reduce((sum, p, i) => sum + p[0] * ring[i + 1][1] - ring[i + 1][0] * p[1], 0) / 2;
const orient = (a, b, c) => (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
const onSegment = (a, b, p) => Math.abs(orient(a, b, p)) <= EPS && p[0] >= Math.min(a[0], b[0]) - EPS && p[0] <= Math.max(a[0], b[0]) + EPS && p[1] >= Math.min(a[1], b[1]) - EPS && p[1] <= Math.max(a[1], b[1]) + EPS;
const intersects = (a, b, c, d) => {
  const ac = orient(a, b, c), ad = orient(a, b, d), ca = orient(c, d, a), cb = orient(c, d, b);
  return ((ac > EPS && ad < -EPS || ac < -EPS && ad > EPS) && (ca > EPS && cb < -EPS || ca < -EPS && cb > EPS)) || onSegment(a, b, c) || onSegment(a, b, d) || onSegment(c, d, a) || onSegment(c, d, b);
};
const inside = (p, ring, allowEdge = false) => {
  let hit = false;
  for (let i = 0, j = ring.length - 2; i < ring.length - 1; j = i++) {
    const a = ring[j], b = ring[i];
    if (onSegment(a, b, p)) return allowEdge;
    if ((a[1] > p[1]) !== (b[1] > p[1]) && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0]) hit = !hit;
  }
  return hit;
};
const ringsIntersect = (a, b) => a.slice(0, -1).some((p, i) => b.slice(0, -1).some((q, j) => intersects(p, a[i + 1], q, b[j + 1])));
export const transformPixel = (matrix, pixel) => [matrix[0] * pixel[0] + matrix[1] * pixel[1] + matrix[2], matrix[3] * pixel[0] + matrix[4] * pixel[1] + matrix[5]];
const inversePoint = (matrix, field) => {
  const d = matrix[0] * matrix[4] - matrix[1] * matrix[3];
  return [(matrix[4] * (field[0] - matrix[2]) - matrix[1] * (field[1] - matrix[5])) / d, (-matrix[3] * (field[0] - matrix[2]) + matrix[0] * (field[1] - matrix[5])) / d];
};
function unicodeText(value) {
  for(let i=0;i<value.length;i++) {
    const c=value.charCodeAt(i);
    if(c>=0xD800&&c<=0xDBFF){const next=value.charCodeAt(++i);if(!(next>=0xDC00&&next<=0xDFFF))throw new Error('Unpaired Unicode surrogate rejected');}
    else if(c>=0xDC00&&c<=0xDFFF)throw new Error('Unpaired Unicode surrogate rejected');
  }
  return JSON.stringify(value);
}
function compareCodePoints(a,b) {
  const aa=Array.from(a,c=>c.codePointAt(0)),bb=Array.from(b,c=>c.codePointAt(0));
  for(let i=0;i<Math.min(aa.length,bb.length);i++)if(aa[i]!==bb[i])return aa[i]-bb[i];
  return aa.length-bb.length;
}
export function canonicalFieldMapContent(document) {
  function encode(value) {
    if (value === null || typeof value === 'boolean') return JSON.stringify(value);
    if (typeof value === 'number') {
      if (!Number.isFinite(value)) throw new Error('Canonical digest rejects non-finite numbers');
      const bytes = new ArrayBuffer(8); new DataView(bytes).setFloat64(0, Object.is(value, -0) ? 0 : value, false);
      return JSON.stringify('f64:' + Array.from(new Uint8Array(bytes), byte => byte.toString(16).padStart(2, '0')).join(''));
    }
    if (typeof value === 'string') return unicodeText(value);
    if (Array.isArray(value)) return '[' + value.map(encode).join(',') + ']';
    if (!value || typeof value !== 'object') throw new Error('Unsupported canonical value');
    return '{' + Object.keys(value).sort(compareCodePoints).map(key => unicodeText(key) + ':' + encode(value[key])).join(',') + '}';
  }
  const content = Object.fromEntries(Object.entries(document).filter(([key]) => key !== 'approval'));
  return encode(content);
}
export async function sha256(bytes) {
  const result = await crypto.subtle.digest('SHA-256', bytes);
  return Array.from(new Uint8Array(result), byte => byte.toString(16).padStart(2, '0')).join('');
}
export const canonicalContentDigest = document => sha256(new TextEncoder().encode(canonicalFieldMapContent(document)));

function validateGeometryUnchecked(document) {
  const errors = [], fail = message => errors.push(message);
  const text=(value,max,empty=false)=>typeof value==='string'&&(empty||value.length>0)&&value.length<=max;
  const nullableText=(value,max)=>value===null||text(value,max,true);
  const keys = (value, expected, label) => {
    if (!value || typeof value !== 'object' || Array.isArray(value)) { fail(label + ' must be an object'); return false; }
    if (Object.keys(value).length !== expected.length || expected.some(k => !Object.hasOwn(value, k))) fail(label + ' contains missing/unknown fields');
    return true;
  };
  if (!keys(document, ['schema_version','map','image','source','calibration','boundary','obstacles','approval'], 'document')) return errors;
  if (document.schema_version !== FIELD_MAP_VERSION) fail('Unsupported schema_version');
  const {map, image, calibration: c, boundary, approval} = document;
  if (!keys(map, ['id','revision','season','variant','frame','units','width_m','height_m'], 'map')) return errors;
  if (!text(map.id,128)||!text(map.variant,128)) fail('Invalid map identity');
  if (!nullableText(map.season,1024)) fail('Invalid season');
  if (!Number.isSafeInteger(map.revision) || map.revision < 1) fail('Invalid map revision');
  if (map.frame !== 'wpilib_nwu' || map.units !== 'm') fail('Only wpilib_nwu metres supported');
  if (![map.width_m,map.height_m].every(v => finite(v) && v > 0 && v <= 100)) fail('Field extents must be finite and 0..100 m');
  if (!keys(image, ['file_name','sha256','width_px','height_px','mime_type','attribution','license'], 'image')) return errors;
  if (typeof image.file_name !== 'string' || image.file_name.length > 128 || !/^[^/\\:]+\.(png|jpe?g|webp)$/i.test(image.file_name) || image.file_name.startsWith('.')) fail('Image must be a local raster basename');
  if (!/^[0-9a-f]{64}$/.test(image.sha256)) fail('Invalid image content hash');
  if (![image.width_px,image.height_px].every(v => Number.isSafeInteger(v) && v > 0 && v <= MAX_DIM) || image.width_px * image.height_px > MAX_PIXELS) fail('Image dimensions exceed bounds');
  if (!['image/png','image/jpeg','image/webp'].includes(image.mime_type)) fail('Only PNG/JPEG/WebP supported');
  if (!text(image.attribution,1024)||!nullableText(image.license,1024)) fail('Image attribution/license invalid');
  if (keys(document.source,['kind','label','uri'],'source') && (!['synthetic','user_image','official_geometry'].includes(document.source.kind) || !text(document.source.label,1024)||!nullableText(document.source.uri,1024))) fail('Invalid source provenance');
  if (!keys(c,['model','image_to_field','control_points','distortion','fit_error_m','independent_check_error_m','independent_check_points'],'calibration')) return errors;
  const m = c.image_to_field;
  if (c.model !== 'affine' || !Array.isArray(m) || m.length !== 9 || !m.every(finite) || m[6] !== 0 || m[7] !== 0 || m[8] !== 1 || (!finite(m[0] * m[4] - m[1] * m[3]) || Math.abs(m[0] * m[4] - m[1] * m[3]) < 1e-12)) { fail('Nondegenerate affine transform required; perspective is unsupported'); return errors; }
  if (!['uncorrected','corrected','not_applicable'].includes(c.distortion)) fail('Invalid lens distortion status');
  for (const name of ['fit_error_m','independent_check_error_m']) if (!(c[name] === null || finite(c[name]) && c[name] >= 0)) fail('Invalid ' + name);
  const validatePoints = (points, label, min) => {
    if (!Array.isArray(points) || points.length < min || points.length > 32) { fail(label + ' count invalid'); return; }
    const seen=new Set();
    for (const p of points) {
      if (!keys(p,['pixel','field_m'],label) || ![p.pixel,p.field_m].every(v => Array.isArray(v) && v.length === 2 && v.every(finite))) { fail(label + ' invalid point'); continue; }
      const identity=JSON.stringify(p.pixel);if(seen.has(identity))fail(label+' needs distinct pixels');seen.add(identity);
      if (!transformPixel(m,p.pixel).every(finite)) fail(label + ' non-finite projection');
      if (p.pixel[0] < 0 || p.pixel[0] > image.width_px || p.pixel[1] < 0 || p.pixel[1] > image.height_px || p.field_m[0] < 0 || p.field_m[0] > map.width_m || p.field_m[1] < 0 || p.field_m[1] > map.height_m) fail(label + ' outside extents');
    }
  };
  validatePoints(c.control_points,'control_points',2); validatePoints(c.independent_check_points,'independent_check_points',0);
  if (Array.isArray(c.control_points) && c.control_points.length >= 2 && c.control_points.every(p => Array.isArray(p.pixel))) {
    if (c.control_points.every(p => same(p.pixel,c.control_points[0].pixel))) fail('Calibration needs distinct known floor references');
  }
  const residual = points => Math.sqrt(points.reduce((sum,p) => {const q=transformPixel(m,p.pixel);return sum+(q[0]-p.field_m[0])**2+(q[1]-p.field_m[1])**2;},0)/points.length);
  if(!errors.length) {
    if(c.fit_error_m!==null && Math.abs(c.fit_error_m-residual(c.control_points))>1e-8)fail('fit_error_m differs from RMS projected residual');
    if(c.independent_check_error_m!==null && (!c.independent_check_points.length || Math.abs(c.independent_check_error_m-residual(c.independent_check_points))>1e-8))fail('independent_check_error_m differs from RMS projected residual');
  }
  const provenance = (p,label) => {
    if (keys(p,['kind','label','uri'],label) && (!['suggested','manual','verified_source'].includes(p.kind) || !text(p.label,1024)||!nullableText(p.uri,1024))) fail(label + ' invalid');
  };
  const review = (r,label) => {
    if (keys(r,['state','reviewed_revision'],label) && (!['draft','approved'].includes(r.state) || (r.state === 'approved' ? r.reviewed_revision !== map.revision : r.reviewed_revision !== null))) fail(label + ' must match exact revision');
  };
  if(!Array.isArray(document.obstacles)||document.obstacles.length>64){fail('At most 64 obstacles');return errors;}
  // Reject excess shape work before any quadratic intersection loops.
  let preflightVertices=0;
  for(const p of [boundary,...document.obstacles]) {
    if(!p||!Array.isArray(p.outer)||!Array.isArray(p.holes)||p.holes.length>16){fail('Malformed bounded polygon');return errors;}
    for(const r of [p.outer,...p.holes]){if(!Array.isArray(r)||r.length>257){fail('Malformed bounded ring');return errors;}preflightVertices+=Math.max(0,r.length-1);if(preflightVertices>4096){fail('At most 4096 polygon vertices');return errors;}}
  }
  let totalVertices = 0;
  const ring = (r, winding,label) => {
    if (!Array.isArray(r) || r.length < 4 || r.length > 257 || r.some(p => !Array.isArray(p) || p.length !== 2 || !p.every(finite))) { fail(label + ' needs bounded finite closed ring'); return false; }
    totalVertices += r.length-1;
    if (!same(r[0],r[r.length-1])) fail(label + ' must be explicitly closed');
    if (r.some(p => p[0] < -EPS || p[1] < -EPS || p[0] > map.width_m + EPS || p[1] > map.height_m + EPS)) fail(label + ' outside field');
    if (Math.abs(area(r)) < 1e-8 || Math.sign(area(r)) !== winding) fail(label + (winding > 0 ? ' must have CCW nonzero area' : ' must have CW nonzero area'));
    for (let i=0;i<r.length-1;i++) {
      if (same(r[i],r[i+1])) fail(label + ' has duplicate adjacent vertex');
      const previous=r[(i+r.length-2)%(r.length-1)],a=r[i],b=r[i+1];
      if(Math.abs(orient(previous,a,b))<=EPS&&(a[0]-previous[0])*(b[0]-a[0])+(a[1]-previous[1])*(b[1]-a[1])<0)fail(label+' has overlapping adjacent edges');
      for(let j=i+1;j<r.length-1;j++) if(j !== i+1 && !(i===0 && j===r.length-2) && intersects(r[i],r[i+1],r[j],r[j+1])) fail(label + ' self-intersects');
    }
    return true;
  };
  const polygon = (p,label, obstacle=false) => {
    if (!keys(p,obstacle?['id','outer','holes','review','provenance','vertical_range_m']:['outer','holes','review','provenance'],label)) return false;
    const outerOkay=ring(p.outer,1,label+'.outer');
    if(!Array.isArray(p.holes)||p.holes.length>16) {fail(label+' holes invalid');return false;}
    const validHoles=[];
    p.holes.forEach((h,i)=>{if(ring(h,-1,label+'.holes['+i+']')) validHoles.push(h);});
    if(outerOkay) for(const h of validHoles) if(!inside(h[0],p.outer)||ringsIntersect(h,p.outer)) fail(label+' hole must lie strictly inside outer ring');
    for(let i=0;i<validHoles.length;i++) for(let j=i+1;j<validHoles.length;j++) if(ringsIntersect(validHoles[i],validHoles[j])||inside(validHoles[i][0],validHoles[j])||inside(validHoles[j][0],validHoles[i])) fail(label+' holes overlap');
    review(p.review,label+'.review');provenance(p.provenance,label+'.provenance');
    if(obstacle && (!text(p.id,128))) fail(label+' id invalid');
    if(obstacle && p.vertical_range_m!==null) {
      if(!keys(p.vertical_range_m,['min','max'],label+'.vertical_range_m')||![p.vertical_range_m.min,p.vertical_range_m.max].every(finite)||p.vertical_range_m.min>=p.vertical_range_m.max||Math.abs(p.vertical_range_m.min)>1000||Math.abs(p.vertical_range_m.max)>1000) fail(label+' height range invalid; unknown is null');
    }
    return outerOkay;
  };
  const boundOkay=polygon(boundary,'boundary');
  if(!Array.isArray(document.obstacles)||document.obstacles.length>64){fail('At most 64 obstacles');return errors;}
  const ids=new Set();
  document.obstacles.forEach((p,i)=>{
    const good=polygon(p,'obstacles['+i+']',true);
    if(ids.has(p.id))fail('Duplicate obstacle id');ids.add(p.id);
    if(boundOkay&&good) {
      if(p.outer.some(v=>!inside(v,boundary.outer,true)))fail('Obstacle outside physical boundary');
      for(const h of boundary.holes) if(ringsIntersect(p.outer,h)||inside(p.outer[0],h,true)||inside(h[0],p.outer,true))fail('Obstacle overlaps excluded boundary hole');
      // A chord can leave a concave boundary although its vertices are inside.
      for(let j=0;j<p.outer.length-1;j++) {
        const midpoint=[(p.outer[j][0]+p.outer[j+1][0])/2,(p.outer[j][1]+p.outer[j+1][1])/2];
        if(!inside(midpoint,boundary.outer,true))fail('Obstacle edge leaves boundary');
        for(let k=0;k<boundary.outer.length-1;k++) {
          const a=p.outer[j],b=p.outer[j+1],q=boundary.outer[k],r=boundary.outer[k+1];
          if(intersects(a,b,q,r)&&!onSegment(q,r,a)&&!onSegment(q,r,b))fail('Obstacle edge crosses boundary');
        }
      }
    }
  });
  if(totalVertices>4096)fail('At most 4096 polygon vertices');
  if(keys(approval,['state','reviewed_revision','content_sha256'],'approval')) {
    if(!['draft','approved'].includes(approval.state))fail('Invalid approval state');
    if(approval.state==='draft'&&(approval.reviewed_revision!==null||approval.content_sha256!==null))fail('Draft map cannot retain approval');
    if(approval.state==='approved') {
      if(approval.reviewed_revision!==map.revision||!/^[0-9a-f]{64}$/.test(approval.content_sha256))fail('Approval must bind revision and digest');
      if([boundary,...document.obstacles].some(p=>p.review?.state!=='approved'||p.review?.reviewed_revision!==map.revision))fail('Every physical polygon needs review');
      if(c.fit_error_m===null||c.independent_check_error_m===null||!c.independent_check_points?.length)fail('Approval requires fit and independent check');
      if(c.independent_check_points?.some(p=>c.control_points?.some(q=>same(p.pixel,q.pixel))))fail('Independent checks must differ from fitted points');
    }
  }
  return [...new Set(errors)];
}
export function validateFieldMapGeometry(document) {
  try { return validateGeometryUnchecked(document); }
  catch { return ['Malformed field-map shape or geometry']; }
}
export async function validateFieldMap(document) {
  const errors=validateFieldMapGeometry(document);
  if(!errors.length) {
    try {
      const digest=await canonicalContentDigest(document);
      if(document.approval.state==='approved'&&digest!==document.approval.content_sha256)errors.push('Approval digest differs from content; review again');
    } catch(error) { errors.push(error.message); }
  }
  return {valid:errors.length===0,errors};
}
export function invalidateFieldMap(document) {
  document.map.revision++;
  document.approval={state:'draft',reviewed_revision:null,content_sha256:null};
  for(const p of [document.boundary,...document.obstacles])p.review={state:'draft',reviewed_revision:null};
  return document;
}

/* Probe encoded dimensions before any browser decode. Animated files are not accepted. */
export function inspectRaster(bytes) {
  const b=bytes instanceof Uint8Array?bytes:new Uint8Array(bytes), d=new DataView(b.buffer,b.byteOffset,b.byteLength);
  if(!b.length||b.length>MAX_BYTES)throw new Error('Raster must be 1 byte to 8 MiB');
  const ascii=(i,n)=>String.fromCharCode(...b.slice(i,i+n));
  let width,height,mime;
  if(b.length>=33&&[137,80,78,71,13,10,26,10].every((v,i)=>b[i]===v)&&ascii(12,4)==='IHDR') {
    width=d.getUint32(16);height=d.getUint32(20);mime='image/png';
    for(let i=8;i+12<=b.length;) {const n=d.getUint32(i);if(n>b.length-i-12)throw new Error('Truncated PNG');if(ascii(i+4,4)==='acTL')throw new Error('Animated PNG unsupported');i+=n+12;}
  } else if(b.length>=12&&ascii(0,4)==='RIFF'&&ascii(8,4)==='WEBP') {
    mime='image/webp';
    for(let i=12;i+8<=b.length;){const kind=ascii(i,4), n=d.getUint32(i+4,true), p=i+8;if(n>b.length-p)throw new Error('Truncated WebP');
      if(kind==='ANIM'||kind==='ANMF')throw new Error('Animated WebP unsupported');
      if(kind==='VP8X'&&n>=10){if(b[p]&2)throw new Error('Animated WebP unsupported');width=1+b[p+4]+(b[p+5]<<8)+(b[p+6]<<16);height=1+b[p+7]+(b[p+8]<<8)+(b[p+9]<<16);}
      if(kind==='VP8 '&&n>=10&&b[p+3]===157&&b[p+4]===1&&b[p+5]===42){width=d.getUint16(p+6,true)&16383;height=d.getUint16(p+8,true)&16383;}
      if(kind==='VP8L'&&n>=5&&b[p]===47){width=1+((b[p+1]|b[p+2]<<8)&16383);height=1+((b[p+2]>>6|b[p+3]<<2|b[p+4]<<10)&16383);}
      i=p+n+(n&1);
    }
  } else if(b.length>=4&&b[0]===255&&b[1]===216) {
    mime='image/jpeg';
    for(let i=2;i+4<b.length;) {
      if(b[i]!==255)throw new Error('Malformed JPEG marker');while(b[i]===255)i++;const marker=b[i++];
      if(marker===217||marker===218)break;if(marker===1||marker>=208&&marker<=215)continue;
      if(i+2>b.length)throw new Error('Truncated JPEG');const n=d.getUint16(i);if(n<2||i+n>b.length)throw new Error('Truncated JPEG segment');
      if(marker===225&&ascii(i+2,6)==='Exif\0\0') {
        const base=i+8;if(base+8<=i+n){const le=ascii(base,2)==='II',offset=d.getUint32(base+4,le),ifd=base+offset;if(ifd+2<=i+n){const count=d.getUint16(ifd,le);for(let j=0;j<count;j++){const q=ifd+2+12*j;if(q+12>i+n)break;if(d.getUint16(q,le)===274&&d.getUint16(q+8,le)!==1)throw new Error('Rotate/export JPEG pixels first; EXIF orientation unsupported');}}}
      }
      if([192,193,194,195,197,198,199,201,202,203,205,206,207].includes(marker)&&n>=8){height=d.getUint16(i+3);width=d.getUint16(i+5);}
      i+=n;
    }
  }
  if(!width||!height||!mime)throw new Error('Only bounded static PNG/JPEG/WebP rasters supported');
  if(width>MAX_DIM||height>MAX_DIM||width*height>MAX_PIXELS)throw new Error('Raster exceeds 8192 px per axis or 16 megapixels');
  return {width,height,mime};
}

export function suggestRegions(rgba,width,height,threshold,minPixels,maxCandidates=32) {
  if(!Number.isInteger(width)||!Number.isInteger(height)||width<1||height<1||width*height>262144||rgba.length!==width*height*4)throw new Error('Suggestion raster limit exceeded');
  const visited=new Uint8Array(width*height),queue=new Int32Array(width*height),boxes=[];
  const dark=i=>rgba[i*4+3]>127&&(rgba[i*4]*.2126+rgba[i*4+1]*.7152+rgba[i*4+2]*.0722)<threshold;
  for(let start=0;start<visited.length;start++) {
    if(visited[start]||!dark(start))continue;
    let head=0,tail=1,count=0,minX=width,maxX=0,minY=height,maxY=0;queue[0]=start;visited[start]=1;
    while(head<tail){const i=queue[head++],x=i%width,y=Math.floor(i/width);count++;minX=Math.min(minX,x);maxX=Math.max(maxX,x);minY=Math.min(minY,y);maxY=Math.max(maxY,y);
      for(const n of [x>0?i-1:-1,x+1<width?i+1:-1,y>0?i-width:-1,y+1<height?i+width:-1])if(n>=0&&!visited[n]&&dark(n)){visited[n]=1;queue[tail++]=n;}
    }
    if(count>=minPixels)boxes.push({minX,minY,maxX:maxX+1,maxY:maxY+1,pixels:count});
  }
  return boxes.sort((a,b)=>b.pixels-a.pixels).slice(0,maxCandidates);
}

export function mountFieldImporter(container,{onSave=()=>{},getMode=()=> 'sandbox'}={}) {
  if(!container)return null;
  container.innerHTML=`<details class="fi-shell"><summary>Import field picture · local sandbox only</summary><p>Import → calibrate → suggest outlines → review → save. Raster pixels never leave this browser. Top-down affine calibration only; perspective photos and elevated AprilTag points are unsupported.</p>
    <div class="fi-actions"><label>Picture <input data-fi="image" type="file" accept="image/png,image/jpeg,image/webp"></label><label>Map JSON <input data-fi="json" type="file" accept="application/json,.json"></label><button data-fi="synthetic">Load synthetic diagram</button></div>
    <p data-fi="message" role="status" aria-live="polite">No image loaded. Upload a top-down diagram or use the clearly synthetic sample.</p>
    <canvas data-fi="canvas" width="800" height="400" aria-label="Image calibration and editable physical polygon preview"></canvas>
    <fieldset><legend>Known floor-plane calibration · image coordinates u right, v down</legend><div class="fi-grid">
      <label>Field +X extent (m)<input data-fi="width" type="number" value="8" min="0.1" max="100" step="0.1"></label><label>Field +Y extent (m)<input data-fi="height" type="number" value="4" min="0.1" max="100" step="0.1"></label>
      <label>Origin u (px)<input data-fi="ou" type="number" value="0"></label><label>Origin v (px)<input data-fi="ov" type="number" value="200"></label>
      <label>+X reference u (px)<input data-fi="ru" type="number" value="400"></label><label>+X reference v (px)<input data-fi="rv" type="number" value="200"></label><label>Known origin→reference distance (m)<input data-fi="distance" type="number" value="8" step="0.01" min="0.01"></label>
      <label>Independent u (px)<input data-fi="cu" type="number" value="200"></label><label>Independent v (px)<input data-fi="cv" type="number" value="100"></label><label>Independent field X (m)<input data-fi="cx" type="number" value="4" step="0.01"></label><label>Independent field Y (m)<input data-fi="cy" type="number" value="2" step="0.01"></label>
      <label>Lens distortion<select data-fi="distortion"><option value="uncorrected">Uncorrected</option><option value="corrected">Corrected externally</option><option value="not_applicable">Not applicable: diagram</option></select></label>
      <label>Attribution<input data-fi="attribution" value="User supplied local image"></label><label>License, if known<input data-fi="license" placeholder="Unknown"></label></div>
      <p>The reference defines +X direction; +Y is on its left on the physical floor. The origin is field (0,0). Verify this orientation explicitly. Independent check must differ from the two fitted points. Lens correction is not performed here. Applying calibration resets the boundary to the declared rectangle; existing obstacles retain their metric coordinates and require review again.</p>
      <label><input data-fi="orientation" type="checkbox"> I verified origin, scale, +X direction and +Y side using known floor references.</label> <button data-fi="calibrate">Apply calibration</button></fieldset>
    <fieldset><legend>Unreviewed draft suggestions</legend><label>Dark-pixel threshold <input data-fi="threshold" type="number" value="100" min="1" max="254"></label><label>Minimum connected pixels <input data-fi="min-area" type="number" value="60" min="4" max="100000"></label><button data-fi="suggest">Suggest bounding outlines</button><p>Dark connected regions become coarse rectangular drafts. Text, shadows and colors can produce false candidates. Holes are not extracted. This is not recognition of physical obstacles, heights or hidden footprints.</p></fieldset>
    <fieldset><legend>Review raw physical geometry · no robot inflation</legend><select data-fi="selection" aria-label="Selected polygon"></select><button data-fi="add">Add manual obstacle</button><button data-fi="delete">Delete selected obstacle</button>
      <div class="fi-grid"><label>Outer ring · metres, closed CCW<textarea data-fi="outer" rows="3"></textarea></label><label>Hole rings · metres, closed CW<textarea data-fi="holes" rows="3"></textarea></label><label>Known minimum Z (m)<input data-fi="zmin" type="number" step="0.01" placeholder="Unknown"></label><label>Known maximum Z (m)<input data-fi="zmax" type="number" step="0.01" placeholder="Unknown"></label></div>
      <button data-fi="apply-geometry">Apply vertex edits</button><button data-fi="approve-shape">Approve selected polygon at this revision</button><p>Drag orange polygon vertices or edit coordinates. Any image, calibration or geometry edit invalidates every polygon review and whole-map approval. Unknown height is null, never zero.</p><p data-fi="review-state"></p></fieldset>
    <div class="fi-actions"><button data-fi="approve-map">Approve exact map content</button><button data-fi="save">Save approved map to sandbox</button><button data-fi="export">Export map JSON</button><button data-fi="export-image">Save original raster locally</button></div>
    <p>Approval affects this local sandbox configuration only. Static field geometry remains separate from live detections. Boundary holes/nonrectangular boundaries may be unsupported by this planner; obstacle holes may be filled conservatively by its documented adapter.</p></details>`;
  const el=name=>container.querySelector('[data-fi="'+name+'"]'),num=name=>Number(el(name).value),canvas=el('canvas'),ctx=canvas.getContext('2d');
  let map=null,bitmap=null,imageUrl=null,imageBlob=null,selected='boundary',drag=null,drawRect=null,imageVerified=false;
  const message=(text,error=false)=>{el('message').textContent=text;el('message').classList.toggle('fi-error',error);};
  const guard=()=>{if(getMode()!=='sandbox')throw new Error('Field editing is isolated to sandbox mode. Switch to sandbox first.');};
  const selectedPoly=()=>selected==='boundary'?map?.boundary:map?.obstacles.find(o=>o.id===selected);
  const normalize=ring=>{if(!same(ring[0],ring[ring.length-1]))ring.push([...ring[0]]);return ring;};
  const markEdit=()=>{invalidateFieldMap(map);imageVerified=!!bitmap&&imageVerified;refresh();};
  function render() {
    ctx.clearRect(0,0,canvas.width,canvas.height);ctx.fillStyle='#162530';ctx.fillRect(0,0,canvas.width,canvas.height);
    if(!bitmap){ctx.fillStyle='#bbcdd3';ctx.font='18px sans-serif';ctx.fillText('Local picture preview',20,40);return;}
    const scale=Math.min(canvas.width/bitmap.width,canvas.height/bitmap.height),x=(canvas.width-bitmap.width*scale)/2,y=(canvas.height-bitmap.height*scale)/2;drawRect={x,y,scale};ctx.drawImage(bitmap,x,y,bitmap.width*scale,bitmap.height*scale);
    if(!map)return;
    const m=map.calibration.image_to_field;
    for(const [name,p] of [['boundary',map.boundary],...map.obstacles.map(o=>[o.id,o])])for(const r of [p.outer,...p.holes]){
      ctx.beginPath();r.forEach((v,i)=>{const q=inversePoint(m,v),u=x+q[0]*scale,w=y+q[1]*scale;i?ctx.lineTo(u,w):ctx.moveTo(u,w);});ctx.strokeStyle=p.review.state==='approved'?'#7de3bb':'#ffb457';ctx.lineWidth=name===selected?3:1.5;ctx.stroke();
      if(name===selected)for(const v of r.slice(0,-1)){const q=inversePoint(m,v);ctx.beginPath();ctx.arc(x+q[0]*scale,y+q[1]*scale,5,0,Math.PI*2);ctx.fillStyle='#ffb457';ctx.fill();}
    }
    for(const [points,color] of [[map.calibration.control_points,'#4aa8ff'],[map.calibration.independent_check_points,'#ea69ff']])for(const p of points){ctx.beginPath();ctx.arc(x+p.pixel[0]*scale,y+p.pixel[1]*scale,5,0,Math.PI*2);ctx.fillStyle=color;ctx.fill();}
  }
  function refresh() {
    el('selection').replaceChildren();if(!map){render();return;}
    for(const [id,label] of [['boundary','Field boundary'],...map.obstacles.map(o=>[o.id,o.id+' · '+o.review.state])]){const option=document.createElement('option');option.value=id;option.textContent=label;el('selection').append(option);}
    if(selected!=='boundary'&&!map.obstacles.some(o=>o.id===selected))selected='boundary';el('selection').value=selected;
    const p=selectedPoly();el('outer').value=JSON.stringify(p.outer);el('holes').value=JSON.stringify(p.holes);el('zmin').value=p.vertical_range_m?.min??'';el('zmax').value=p.vertical_range_m?.max??'';el('zmin').disabled=selected==='boundary';el('zmax').disabled=selected==='boundary';
    el('review-state').textContent='Map revision '+map.map.revision+' · selected '+p.review.state+' · whole map '+map.approval.state+(imageVerified?' · image hash verified':' · matching image required');render();
  }
  const bind=(name,fn)=>el(name).addEventListener('click',async()=>{try{guard();await fn();}catch(error){message(error.message,true);}});
  function emptyMap(image,synthetic=false) {
    const w=8,h=4,rev={state:'draft',reviewed_revision:null},prov={kind:'manual',label:'Unreviewed local physical geometry',uri:null};
    return {schema_version:FIELD_MAP_VERSION,map:{id:'local-'+crypto.randomUUID(),revision:1,season:null,variant:synthetic?'synthetic':'user-import',frame:'wpilib_nwu',units:'m',width_m:w,height_m:h},image,
      source:{kind:synthetic?'synthetic':'user_image',label:synthetic?'SYNTHETIC TEST DIAGRAM — not official field geometry':'User image — geometry and dimensions require review',uri:null},
      calibration:{model:'affine',image_to_field:[w/image.width_px,0,0,0,-h/image.height_px,h,0,0,1],control_points:[{pixel:[0,image.height_px],field_m:[0,0]},{pixel:[image.width_px,image.height_px],field_m:[w,0]}],distortion:synthetic?'not_applicable':'uncorrected',fit_error_m:null,independent_check_error_m:null,independent_check_points:[]},
      boundary:{outer:[[0,0],[w,0],[w,h],[0,h],[0,0]],holes:[],review:copy(rev),provenance:copy(prov)},obstacles:[],approval:{state:'draft',reviewed_revision:null,content_sha256:null}};
  }
  async function loadImage(file,synthetic=false) {
    guard();if(!file.size||file.size>MAX_BYTES)throw new Error('Raster must be 1 byte to 8 MiB');const bytes=await file.arrayBuffer(),info=inspectRaster(bytes),hash=await sha256(bytes);
    const nextBitmap=await createImageBitmap(new Blob([bytes],{type:info.mime}),{imageOrientation:'none'});
    guard();if(nextBitmap.width!==info.width||nextBitmap.height!==info.height){nextBitmap.close();throw new Error('Decoded image dimensions differ from encoded pixels');}
    bitmap?.close();bitmap=nextBitmap;if(imageUrl)URL.revokeObjectURL(imageUrl);imageBlob=new Blob([bytes],{type:info.mime});imageUrl=URL.createObjectURL(imageBlob);imageVerified=true;
    const sameImage=map&&map.image.sha256===hash&&map.image.width_px===info.width&&map.image.height_px===info.height;
    if(!sameImage){const image={file_name:(file.name.replace(/[^A-Za-z0-9._-]/g,'_').replace(/^\.+/,'').replace(/\.[^.]*$/,'').slice(0,110)||'local-image')+({ 'image/png':'.png','image/jpeg':'.jpg','image/webp':'.webp' }[info.mime]),sha256:hash,width_px:info.width,height_px:info.height,mime_type:info.mime,attribution:synthetic?'Locally generated synthetic diagram':'User supplied local image',license:synthetic?'CC0-1.0':null};const revision=map?map.map.revision+1:1;map=emptyMap(image,synthetic);map.map.revision=revision;}
    el('ou').value=0;el('ov').value=info.height;el('ru').value=info.width;el('rv').value=info.height;el('cu').value=info.width/2;el('cv').value=info.height/2;el('width').value=map.map.width_m;el('height').value=map.map.height_m;el('distance').value=map.map.width_m;el('cx').value=map.map.width_m/2;el('cy').value=map.map.height_m/2;el('distortion').value=map.calibration.distortion;el('attribution').value=map.image.attribution;el('license').value=map.image.license??'';el('orientation').checked=false;
    refresh();message(sameImage?'Matching image content verified; imported metric geometry retained.':(synthetic?'SYNTHETIC sample loaded. ':'Local image loaded. ')+'Calibrate known floor references; initial rectangle is an unreviewed draft.');
  }
  el('image').addEventListener('change',async()=>{try{if(el('image').files[0])await loadImage(el('image').files[0]);}catch(error){message(error.message,true);}el('image').value='';});
  el('json').addEventListener('change',async()=>{try{guard();const f=el('json').files[0];if(!f)return;if(f.size>2*1024*1024)throw new Error('Map JSON exceeds 2 MiB');const doc=JSON.parse(await f.text()),validation=await validateFieldMap(doc);if(!validation.valid)throw new Error(validation.errors.join('; '));map=doc;imageVerified=!!bitmap&&await sha256(await imageBlob.arrayBuffer())===map.image.sha256;selected='boundary';refresh();message('Map imported. '+(imageVerified?'Matching image verified.':'Choose its exact matching raster before approval/save.'));}catch(error){message(error.message,true);}el('json').value='';});
  bind('synthetic',async()=>{const c=document.createElement('canvas');c.width=400;c.height=200;const g=c.getContext('2d');g.fillStyle='#e7f0ee';g.fillRect(0,0,400,200);g.fillStyle='#283446';g.fillRect(150,50,100,100);g.fillStyle='#e7f0ee';g.fillRect(175,75,50,50);const b=await new Promise(resolve=>c.toBlob(resolve,'image/png'));await loadImage(new File([b],'synthetic-local.png',{type:'image/png'}),true);});
  bind('calibrate',()=>{
    if(!map||!bitmap)throw new Error('Load an image first');if(!el('orientation').checked)throw new Error('Confirm floor origin, scale and orientation first');
    const w=num('width'),h=num('height'),ou=num('ou'),ov=num('ov'),ru=num('ru'),rv=num('rv'),distance=num('distance'),dx=ru-ou,dy=rv-ov,length=Math.hypot(dx,dy);
    if(![w,h,ou,ov,ru,rv,distance].every(finite)||w<=0||h<=0||w>100||h>100||distance<=0||length<1e-6)throw new Error('Finite extents and distinct floor references required');
    const scale=distance/length,a=scale*dx/length,b=scale*dy/length,m=[a,b,-a*ou-b*ov,b,-a,-b*ou+a*ov,0,0,1],check={pixel:[num('cu'),num('cv')],field_m:[num('cx'),num('cy')]};
    const field=transformPixel(m,check.pixel),error=Math.hypot(field[0]-check.field_m[0],field[1]-check.field_m[1]),limit=Math.max(.05,Math.hypot(w,h)*.005);
    if(![...check.pixel,...check.field_m,error].every(finite)||same(check.pixel,[ou,ov])||same(check.pixel,[ru,rv]))throw new Error('Independent check must be finite and distinct from fitted pixels');
    if(error>limit)throw new Error('Independent check error '+error.toFixed(3)+' m exceeds '+limit.toFixed(3)+' m; adjust references');
    const next=copy(map);next.map.width_m=w;next.map.height_m=h;next.calibration={model:'affine',image_to_field:m,control_points:[{pixel:[ou,ov],field_m:[0,0]},{pixel:[ru,rv],field_m:[distance,0]}],distortion:el('distortion').value,fit_error_m:0,independent_check_error_m:error,independent_check_points:[check]};next.image.attribution=el('attribution').value.trim();next.image.license=el('license').value.trim()||null;
    next.boundary.outer=[[0,0],[w,0],[w,h],[0,h],[0,0]];next.boundary.holes=[];invalidateFieldMap(next);const errors=validateFieldMapGeometry(next);if(errors.length)throw new Error(errors.join('; '));map=next;refresh();message('Affine calibration applied; independent error '+error.toFixed(3)+' m. Review all physical geometry.');
  });
  bind('suggest',()=>{
    if(!map||!bitmap||map.calibration.fit_error_m===null)throw new Error('Apply calibrated floor references first');
    const threshold=num('threshold'),min=num('min-area');if(!finite(threshold)||threshold<1||threshold>254||!Number.isInteger(min)||min<4||min>100000)throw new Error('Threshold/minimum pixels outside bounds');
    const c=document.createElement('canvas'),scale=Math.min(1,512/Math.max(bitmap.width,bitmap.height));c.width=Math.max(1,Math.round(bitmap.width*scale));c.height=Math.max(1,Math.round(bitmap.height*scale));const g=c.getContext('2d',{willReadFrequently:true});g.drawImage(bitmap,0,0,c.width,c.height);const boxes=suggestRegions(g.getImageData(0,0,c.width,c.height).data,c.width,c.height,threshold,min,Math.max(0,64-map.obstacles.length));
    let added=0;const next=copy(map);for(const box of boxes){const ring=[[box.minX,box.minY],[box.maxX,box.minY],[box.maxX,box.maxY],[box.minX,box.maxY]].map(p=>transformPixel(next.calibration.image_to_field,[p[0]*bitmap.width/c.width,p[1]*bitmap.height/c.height]));ring.push([...ring[0]]);if(area(ring)<0)ring.reverse();if(ring.some(p=>p[0]<0||p[1]<0||p[0]>map.map.width_m||p[1]>map.map.height_m))continue;next.obstacles.push({id:'draft-'+crypto.randomUUID().slice(0,8),outer:ring,holes:[],review:{state:'draft',reviewed_revision:null},provenance:{kind:'suggested',label:'Dark-region bounding outline; unverified physical footprint, holes not extracted',uri:null},vertical_range_m:null});added++;}
    invalidateFieldMap(next);const errors=validateFieldMapGeometry(next);if(errors.length)throw new Error(errors.join('; '));map=next;refresh();message(added+' coarse outline drafts added. Review/remove text, shadows and nonphysical regions.');
  });
  el('selection').addEventListener('change',()=>{selected=el('selection').value;refresh();});
  bind('add',()=>{if(!map)throw new Error('Load/calibrate image first');if(map.obstacles.length>=64)throw new Error('Obstacle limit reached');const x=map.map.width_m/2,y=map.map.height_m/2,dx=Math.min(.25,x/2),dy=Math.min(.25,y/2),id='manual-'+crypto.randomUUID().slice(0,8);map.obstacles.push({id,outer:[[x-dx,y-dy],[x+dx,y-dy],[x+dx,y+dy],[x-dx,y+dy],[x-dx,y-dy]],holes:[],review:{state:'draft',reviewed_revision:null},provenance:{kind:'manual',label:'User edited local physical footprint',uri:null},vertical_range_m:null});selected=id;markEdit();message('Manual obstacle draft added. Drag vertices or edit coordinates.');});
  bind('delete',()=>{if(!map||selected==='boundary')throw new Error('Select an obstacle to delete');map.obstacles=map.obstacles.filter(o=>o.id!==selected);selected='boundary';markEdit();message('Obstacle deleted; approvals invalidated.');});
  bind('apply-geometry',()=>{if(!map)throw new Error('No map');const next=copy(map),p=selected==='boundary'?next.boundary:next.obstacles.find(o=>o.id===selected);p.outer=normalize(JSON.parse(el('outer').value));p.holes=JSON.parse(el('holes').value).map(normalize);p.provenance={kind:'manual',label:'User edited/reviewed local physical footprint',uri:null};if(selected!=='boundary'){const a=el('zmin').value,b=el('zmax').value;if(!!a!==!!b)throw new Error('Both known Z limits required, or leave both unknown');p.vertical_range_m=a&&b?{min:Number(a),max:Number(b)}:null;}invalidateFieldMap(next);const errors=validateFieldMapGeometry(next);if(errors.length)throw new Error(errors.join('; '));map=next;refresh();message('Geometry edited; every approval invalidated.');});
  bind('approve-shape',()=>{if(!map)throw new Error('No map');const errors=validateFieldMapGeometry(map);if(errors.length)throw new Error(errors.join('; '));selectedPoly().review={state:'approved',reviewed_revision:map.map.revision};map.approval={state:'draft',reviewed_revision:null,content_sha256:null};refresh();message('Selected physical polygon reviewed at revision '+map.map.revision+'.');});
  bind('approve-map',async()=>{if(!map||!imageVerified)throw new Error('Matching image content must be verified');if(map.calibration.fit_error_m===null||map.calibration.independent_check_error_m===null||!map.calibration.independent_check_points.length)throw new Error('Calibrate and independently check floor references first');if([map.boundary,...map.obstacles].some(p=>p.review.state!=='approved'||p.review.reviewed_revision!==map.map.revision))throw new Error('Approve every boundary/obstacle polygon at this revision');map.approval={state:'approved',reviewed_revision:map.map.revision,content_sha256:await canonicalContentDigest(map)};const v=await validateFieldMap(map);if(!v.valid){map.approval={state:'draft',reviewed_revision:null,content_sha256:null};throw new Error(v.errors.join('; '));}refresh();message('Exact map revision/content approved for local sandbox only.');});
  bind('save',async()=>{if(!map||!imageVerified||map.approval.state!=='approved')throw new Error('Approve exact map content and verify its image first');const v=await validateFieldMap(map);if(!v.valid)throw new Error(v.errors.join('; '));guard();await onSave(copy(map),imageUrl);message('Approved raw physical field map saved to sandbox. Live state remains separate.');});
  const download=(blob,name)=>{const url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
  bind('export',()=>{if(!map)throw new Error('No map to export');download(new Blob([JSON.stringify(map,null,2)+'\n'],{type:'application/json'}),map.map.id+'.json');message('Map JSON exported locally. Keep the matching raster with it.');});
  bind('export-image',()=>{if(!imageBlob||!map)throw new Error('No matching image');download(imageBlob,map.image.file_name);message('Original raster offered as local download.');});
  canvas.addEventListener('pointerdown',event=>{try{guard();if(!map||!drawRect)return;const rect=canvas.getBoundingClientRect(),pixel=[(event.clientX-rect.left)*canvas.width/rect.width,(event.clientY-rect.top)*canvas.height/rect.height],p=selectedPoly();for(const [ri,ring] of [p.outer,...p.holes].entries())for(let i=0;i<ring.length-1;i++){const q=inversePoint(map.calibration.image_to_field,ring[i]),x=drawRect.x+q[0]*drawRect.scale,y=drawRect.y+q[1]*drawRect.scale;if(Math.hypot(x-pixel[0],y-pixel[1])<12){drag={ring:ri,index:i,original:copy(map)};canvas.setPointerCapture(event.pointerId);return;}}}catch(error){message(error.message,true);}});
  canvas.addEventListener('pointermove',event=>{if(!drag)return;try{guard();const rect=canvas.getBoundingClientRect(),u=((event.clientX-rect.left)*canvas.width/rect.width-drawRect.x)/drawRect.scale,v=((event.clientY-rect.top)*canvas.height/rect.height-drawRect.y)/drawRect.scale,p=transformPixel(map.calibration.image_to_field,[u,v]),poly=selectedPoly(),ring=drag.ring===0?poly.outer:poly.holes[drag.ring-1];ring[drag.index]=p;if(drag.index===0)ring[ring.length-1]=[...p];render();}catch(error){map=drag.original;drag=null;refresh();message(error.message,true);}});
  const finishDrag=()=>{if(!drag)return;const original=drag.original;drag=null;if(getMode()!=='sandbox'){map=original;refresh();message('Vertex edit cancelled outside sandbox',true);return;}if(JSON.stringify(original)===JSON.stringify(map)){refresh();return;}invalidateFieldMap(map);const errors=validateFieldMapGeometry(map);if(errors.length){map=original;message('Vertex edit rejected: '+errors.join('; '),true);}else message('Vertex edited; all approvals invalidated.');refresh();};
  canvas.addEventListener('pointerup',finishDrag);canvas.addEventListener('pointercancel',()=>{if(drag){map=drag.original;drag=null;refresh();}});
  render();return {getMap:()=>map?copy(map):null,refresh,dispose(){bitmap?.close();if(imageUrl)URL.revokeObjectURL(imageUrl);container.replaceChildren();}};
}
if(typeof window!=='undefined')window.FieldImport={init:mountFieldImporter,mountFieldImporter,validateFieldMap,validateFieldMapGeometry,canonicalContentDigest,canonicalFieldMapContent,transformPixel};
