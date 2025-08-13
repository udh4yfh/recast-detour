const RC_MESH_NULL_IDX: u16 = 0xffff;

// A (nearly) direct port of rcBuildMeshAdjacency from RecastMesh.cpp.
// pub fn build_mesh_adjacency(polys: &mut [u16], vertex_count: usize, vertices_per_poly: usize) {
//     let poly_stride = vertices_per_poly * 2;
//     let poly_count = polys.len() / poly_stride;

//     let max_edge_count = polys.len() * vertices_per_poly;
//     let mut edge_indices = vec![0; vertex_count + max_edge_count];
//     let (first_edge, next_edge) = edge_indices.split_at_mut(vertex_count);

//     #[allow(non_snake_case)]
//     struct Edge {
//         vert: [u16; 2],
//         polyEdge: [u16; 2],
//         poly: [u16; 2],
//     }
//     let mut edges = Vec::with_capacity(max_edge_count);

//     first_edge.fill(RC_MESH_NULL_IDX);
//     for i in 0..poly_count {
//         let t = &polys[i * poly_stride..(i + 1) * poly_stride];
//         for j in 0..vertices_per_poly {
//             if t[j] == RC_MESH_NULL_IDX {
//                 break;
//             }
//             let v0 = t[j];
//             let v1 = if j + 1 >= vertices_per_poly || t[j + 1] == RC_MESH_NULL_IDX {
//                 t[0]
//             } else {
//                 t[j + 1]
//             };
//             if v0 < v1 {
//                 edges.push(Edge {
//                     vert: [v0, v1],
//                     poly: [i as u16, i as u16],
//                     polyEdge: [j as u16, 0],
//                 });
//                 next_edge[edges.len() - 1] = first_edge[v0 as usize];
//                 first_edge[v0 as usize] = (edges.len() - 1) as u16;
//             }
//         }
//     }

//     for i in 0..poly_count {
//         let t = &polys[i * poly_stride..(i + 1) * poly_stride];
//         for j in 0..vertices_per_poly {
//             if t[j] == RC_MESH_NULL_IDX {
//                 break;
//             }
//             let v0 = t[j];
//             let v1 = if j + 1 >= vertices_per_poly || t[j + 1] == RC_MESH_NULL_IDX {
//                 t[0]
//             } else {
//                 t[j + 1]
//             };
//             if v0 > v1 {
//                 let mut e = first_edge[v1 as usize];
//                 while e != RC_MESH_NULL_IDX {
//                     let edge = &mut edges[e as usize];
//                     if edge.vert[1] == v0 && edge.poly[0] == edge.poly[1] {
//                         edge.poly[1] = i as u16;
//                         edge.polyEdge[1] = j as u16;
//                         break;
//                     }
//                     e = next_edge[e as usize];
//                 }
//             }
//         }
//     }

//     for e in edges.iter() {
//         if e.poly[0] != e.poly[1] {
//             let p0_start = e.poly[0] as usize * poly_stride;
//             let p1_start = e.poly[1] as usize * poly_stride;

//             polys[p0_start + vertices_per_poly + e.polyEdge[0] as usize] = e.poly[1];
//             polys[p1_start + vertices_per_poly + e.polyEdge[1] as usize] = e.poly[0];
//         }
//     }
// }

#[derive(Clone)]
struct RcEdge {
	vert: [u16; 2],
	poly_edge: [u16; 2],
	poly: [u16; 2]
}

// A (nearly) direct port of rcBuildMeshAdjacency from RecastMesh.cpp.
pub fn build_mesh_adjacency(polys: &mut [u16], npolys: usize, nverts: usize, verts_per_poly: usize) {
    let max_edge_count = npolys*verts_per_poly;

    let mut edge_indices: Vec<u16> = vec![0; nverts + max_edge_count];
    let (first_edge, next_edge) = edge_indices.split_at_mut(nverts);

	let mut edge_count: usize = 0;
    let mut edges: Vec<RcEdge> = vec!(RcEdge { vert:[0,0], poly_edge: [0,0], poly: [0,0] };  max_edge_count);

    for i in 0..nverts {
		first_edge[i] = RC_MESH_NULL_IDX;
    }

    for i in 0..npolys {
		let t: &[u16] = &polys[i*verts_per_poly*2..];
		for j in 0..verts_per_poly {
			if t[j] == RC_MESH_NULL_IDX { break; }
			let v0 = t[j];
			let v1 = if j+1 >= verts_per_poly || t[j+1] == RC_MESH_NULL_IDX { t[0] } else { t[j+1] };
			if v0 < v1 {
				let edge = &mut edges[edge_count];
				edge.vert[0] = v0;
				edge.vert[1] = v1;
				edge.poly[0] = i as u16;
				edge.poly_edge[0] = j as u16;
				edge.poly[1] = i as u16;
				edge.poly_edge[1] = 0;
				// Insert edge
				next_edge[edge_count] = first_edge[v0 as usize];
				first_edge[v0 as usize] = edge_count as u16;
				edge_count = edge_count + 1;
			}
		}
	}

    for i in 0..npolys {
        let t: &[u16] = &polys[i*verts_per_poly*2..];
		for j in 0..verts_per_poly {
			if t[j] == RC_MESH_NULL_IDX { break };
			let v0 = t[j];
			let v1 = if j+1 >= verts_per_poly || t[j+1] == RC_MESH_NULL_IDX { t[0] } else { t[j+1] };
			if v0 > v1 {
                let mut e = first_edge[v1 as usize];
                while e != RC_MESH_NULL_IDX {
                    let edge = &mut edges[e as usize];
					if edge.vert[1] == v0 && edge.poly[0] == edge.poly[1] {
						edge.poly[1] = i as u16;
						edge.poly_edge[1] = j as u16;
						break;
					}
                    e = next_edge[e as usize];
                }
			}
		}
	}

    // Store adjacency
	for i in 0..edge_count {
        let e = &edges[i];
		if e.poly[0] != e.poly[1] {
			let p0 = &mut polys[e.poly[0] as usize * verts_per_poly*2..];
			p0[verts_per_poly + e.poly_edge[0] as usize] = e.poly[1];
			let p1 = &mut polys[e.poly[1] as usize * verts_per_poly*2..];
			p1[verts_per_poly + e.poly_edge[1] as usize] = e.poly[0];
		}
	}
}